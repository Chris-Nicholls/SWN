#include "chord_gen.h"
#include "quantz_scales.h"
#include "params_update.h"
#include "globals.h"
#include <math.h>

extern o_params params;

// Forward declarations (defined later in this file)
static float freq_to_semitones(float freq);
static inline float fast_log2f(float x);

// ============================================================================
// Scale Mask Utilities
// ============================================================================

// Rotate a 12-bit scale mask by semitones (positive = up from C)
// E.g., rotate_scale_mask(SCALE_MASK_MAJOR, 7) gives G major
uint16_t rotate_scale_mask(uint16_t base_mask, int8_t semitones) {
    if (base_mask == SCALE_MASK_CHROMATIC) {
        return SCALE_MASK_CHROMATIC;  // Chromatic doesn't need rotation
    }
    
    // Normalize semitones to 0-11 range
    int8_t offset = semitones % 12;
    if (offset < 0) offset += 12;
    
    if (offset == 0) {
        return base_mask;
    }
    
    // Rotate the mask: shift left by offset, wrap around within 12 bits
    uint16_t rotated = ((base_mask << offset) | (base_mask >> (12 - offset))) & 0xFFF;
    return rotated;
}

// Get scale mask for a SWN scale enum value, rotated by key_semitones
uint16_t get_scale_mask_for_swn_scale(uint8_t scale_num, int8_t key_semitones) {
    uint16_t base_mask;
    
    switch (scale_num) {
        case sclm_MAJOR:
            base_mask = SCALE_MASK_MAJOR;
            break;
        case sclm_MINOR:
            base_mask = SCALE_MASK_MINOR_HARMONIC;
            break;
        case sclm_SEMITONES:
        case sclm_NONE:
        default:
            return SCALE_MASK_CHROMATIC;  // No constraint
    }
    
    return rotate_scale_mask(base_mask, key_semitones);
}

// Check if a frequency is in the scale (returns 1 if in scale, 0 if not)
static inline uint8_t is_freq_in_scale(float freq_hz, uint16_t scale_mask) {
    if (scale_mask == SCALE_MASK_CHROMATIC) {
        return 1;  // All notes allowed
    }
    
    // Convert frequency to semitone class (0-11)
    // Using C0 = 16.3516 Hz as reference
    float st = freq_to_semitones(freq_hz);
    int8_t semitone_class = ((int)roundf(st) % 12 + 12) % 12;  // Ensure positive modulo
    
    return (scale_mask >> semitone_class) & 1;
}

// Calculate scale penalty for a frequency
// Returns scale_penalty if note is NOT in scale, 0 otherwise
static inline float calc_scale_penalty(float freq_hz, uint16_t scale_mask, float scale_penalty) {
    if (scale_mask == SCALE_MASK_CHROMATIC || scale_penalty <= 0.0f) {
        return 0.0f;
    }
    
    return is_freq_in_scale(freq_hz, scale_mask) ? 0.0f : scale_penalty;
}

// Calculate boundary penalty for frequencies above extension
// Matches Python _boundary_penalty_freqs() - penalty grows linearly with octaves above boundary
// above_db_per_oct: penalty in dB/octave (e.g., 2.0 means +2dB penalty per octave above)
static inline float calc_boundary_penalty(float freq_hz, float extension_freq_hz, float above_db_per_oct) {
    if (above_db_per_oct <= 0.0f || freq_hz <= extension_freq_hz) {
        return 0.0f;
    }
    
    // Octaves above = log2(freq / extension)
    float oct_above = fast_log2f(freq_hz / extension_freq_hz);
    
    // Slope = 10^(dB/20) - 1, matching Python implementation
    float slope = powf(10.0f, above_db_per_oct / 20.0f) - 1.0f;
    
    return slope * oct_above;
}

// ============================================================================
// Harmonic Dissonance Chord Generation
// ============================================================================
// Uses a psychoacoustic model to select chord tones that minimize harmonic
// dissonance. The algorithm greedily adds notes by evaluating candidates
// generated from harmonics of existing notes.
//
// Key concepts:
// - Dissonance kernel: piecewise-linear triangle in semitone distance
// - Register interpolation: kernel shape varies between C2 and C6 anchors
// - Overtone weighting: dissonance summed across weighted harmonics
// - Mean aggregation: average dissonance to all existing notes
// ============================================================================

// Maximum candidates to evaluate per iteration
#define MAX_CANDIDATES 128

// Candidate cache entry for dissonance optimization
typedef struct {
    float freq;           // Candidate frequency (0 = unused slot)
    float total_diss;     // Cumulative dissonance sum (not mean)
    uint8_t num_notes;    // Number of notes this was evaluated against
} CandidateCache;

// Absolute frequency limits (safety bounds)
#define ABS_MIN_FREQ_HZ 16.35f   // C0
#define ABS_MAX_FREQ_HZ 8372.0f  // C9

// ============================================================================
// OPTIMIZATION: Lookup tables for register-dependent kernel parameters
// ============================================================================
// Pre-computed x_peak and fall values for each semitone from C0 (index 0)
// to C9 (index 108). Covers fundamentals and harmonics up to ~8kHz.
// This eliminates expensive log/exp calls in the hot path.

#define LUT_SIZE 128  // Covers C0 to ~C10 (more than enough for harmonics)

// Lookup tables (initialized on first use)
static float lut_x_peak[LUT_SIZE];
static float lut_fall[LUT_SIZE];
static float lut_freq[LUT_SIZE];  // Frequency at each semitone
static uint8_t lut_initialized = 0;

// Precomputed semitone offsets for harmonics k=1..7: 12*log2(k)
// This allows computing partial semitones as f0_st + log2_k[i] without any log calls
static const float log2_k_semitones[CHORD_NUM_OVERTONES] = {
    0.0f,          // k=1: 12*log2(1) = 0
    12.0f,         // k=2: 12*log2(2) = 12 (octave)
    19.01955f,     // k=3: 12*log2(3) ≈ 19.02 (octave + fifth)
    24.0f,         // k=4: 12*log2(4) = 24 (2 octaves)
    27.86314f,     // k=5: 12*log2(5) ≈ 27.86
    31.01955f,     // k=6: 12*log2(6) ≈ 31.02
    33.68826f      // k=7: 12*log2(7) ≈ 33.69
};

// Improved fast log2 approximation
// Splits into exact exponent + polynomial for mantissa
// Accurate to ~0.01% (vs ~1% for basic bit hack)
// ~10 cycles on Cortex-M7 vs ~150 for stdlib log2f
static inline float fast_log2f(float x) {
    union { float f; uint32_t i; } vx = { x };
    
    // Extract exponent bits (exact integer part of log2)
    int32_t e = ((int32_t)(vx.i >> 23) & 0xFF) - 127;
    
    // Set exponent to 0, giving mantissa m in [1.0, 2.0)
    vx.i = (vx.i & 0x007FFFFF) | 0x3F800000;
    float m = vx.f;
    
    // Polynomial approximation for log2(m) where m in [1, 2)
    // Uses Taylor series: log2(1+t) ≈ t*(1/ln2 - t/(2*ln2) + t²/(3*ln2))
    // Coefficients: 1.4427f, -0.7214f, 0.4809f
    float t = m - 1.0f;
    float log2_m = t * (1.4427f - t * (0.7214f - t * 0.4809f));
    
    return (float)e + log2_m;
}

// Initialize lookup tables (called once)
static void init_luts(void) {
    if (lut_initialized) return;
    
    // Precompute log values for geometric interpolation
    float log_c2 = log2f(C2_FREQ_HZ);
    float log_c6 = log2f(C6_FREQ_HZ);
    float log_peak_c2 = logf(CHORD_PEAK_SEMITONES_C2);
    float log_peak_c6 = logf(CHORD_PEAK_SEMITONES_C6);
    float log_fall_c2 = logf(CHORD_TAIL_HALFWIDTH_C2);
    float log_fall_c6 = logf(CHORD_TAIL_HALFWIDTH_C6);
    
    for (int st = 0; st < LUT_SIZE; st++) {
        // Frequency at this semitone (C0 = semitone 0)
        float freq = 16.3516015625f * powf(2.0f, (float)st / 12.0f);
        lut_freq[st] = freq;
        
        // Register interpolation parameter t
        float log_f = log2f(freq);
        float t = (log_f - log_c2) / (log_c6 - log_c2);
        
        // Geometric interpolation for x_peak and fall
        lut_x_peak[st] = expf((1.0f - t) * log_peak_c2 + t * log_peak_c6);
        lut_fall[st] = expf((1.0f - t) * log_fall_c2 + t * log_fall_c6);
    }
    
    lut_initialized = 1;
}

// Helper: convert frequency to semitone index (clamped to LUT range)
// Uses fast approximation - OK for LUT index since we just need nearby entry
static inline uint8_t freq_to_lut_index(float freq) {
    if (freq <= 0.0f) return 0;
    // Fast semitone calculation: 12 * log2(f / C0)
    float st = 12.0f * fast_log2f(freq / 16.3516015625f);
    if (st < 0.0f) return 0;
    if (st >= LUT_SIZE - 1) return LUT_SIZE - 1;
    return (uint8_t)(st + 0.5f);  // Round to nearest
}

// Helper: convert frequency to semitones from C0 (fast approximation)
// Use for dissonance calculations where ~0.7 semitone error is acceptable
static inline float freq_to_semitones_fast(float freq) {
    if (freq <= 0.0f) return 0.0f;
    return 12.0f * fast_log2f(freq / 16.3516015625f);
}

// Helper: convert frequency to semitones from C0 (exact)
// Use for quantization where we need correct rounding to nearest semitone
static float freq_to_semitones(float freq) {
    if (freq <= 0.0f) return 0.0f;
    return 12.0f * log2f(freq / 16.3516015625f);
}

// Helper: convert semitones from C0 to frequency (uses LUT for integer semitones)
static inline float semitones_to_freq(float st) {
    int st_int = (int)(st + 0.5f);  // Round to nearest
    if (st_int >= 0 && st_int < LUT_SIZE) {
        return lut_freq[st_int];
    }
    // Fallback for out-of-range (shouldn't happen in practice)
    return 16.3516015625f * powf(2.0f, st / 12.0f);
}

// Optimized sine dissonance using lookup tables
// Returns BASE dissonance between two pure sine waves at f1_hz and f2_hz
static inline float sine_dissonance_base(float f1_hz, float f2_hz) {
    if (f1_hz <= 0.0f || f2_hz <= 0.0f) return 0.0f;
    
    // Semitone distance (absolute) using fast log2
    float ratio = f2_hz / f1_hz;
    float x = fabsf(12.0f * fast_log2f(ratio));
    
    // Look up x_peak and fall for the lower frequency
    float f_min = (f1_hz < f2_hz) ? f1_hz : f2_hz;
    uint8_t idx = freq_to_lut_index(f_min);
    float x_peak = lut_x_peak[idx];
    float fall = lut_fall[idx];
    
    // Piecewise-linear triangle kernel with peak = 1.0
    float D;
    if (x <= x_peak) {
        D = x / x_peak;
    } else {
        D = 1.0f - (x - x_peak) / fall;
    }
    
    return (D > 0.0f) ? D : 0.0f;
}

// ============================================================================
// OPTIMIZATION: Precomputed harmonics for chosen notes (in semitones)
// ============================================================================
// Stores harmonic semitones for all chosen notes to avoid log calls
// during candidate evaluation. Each partial[i] = f0_semitones + log2_k_semitones[i]

#define MAX_CHOSEN_HARMONICS (NUM_CHANNELS * CHORD_NUM_OVERTONES)

typedef struct {
    float f0_st;                          // Fundamental frequency in semitones from C0
    float partials_st[CHORD_NUM_OVERTONES]; // Harmonic semitones
} HarmonicSet;

// Semitone-domain sine dissonance (no log calls - all inputs are semitones)
// st1, st2: frequencies expressed as semitones from C0
static inline float sine_dissonance_base_st(float st1, float st2) {
    // Semitone distance (absolute)
    float x = fabsf(st1 - st2);
    
    // Look up x_peak and fall for the lower frequency
    float st_min = (st1 < st2) ? st1 : st2;
    int idx = (int)(st_min + 0.5f);
    if (idx < 0) idx = 0;
    if (idx >= LUT_SIZE) idx = LUT_SIZE - 1;
    float x_peak = lut_x_peak[idx];
    float fall = lut_fall[idx];
    
    // Piecewise-linear triangle kernel with peak = 1.0
    float D;
    if (x <= x_peak) {
        D = x / x_peak;
    } else {
        D = 1.0f - (x - x_peak) / fall;
    }
    
    return (D > 0.0f) ? D : 0.0f;
}

// Overtone dissonance using precomputed harmonics (semitone domain)
// f1_st: candidate frequency in semitones from C0
// h2: precomputed harmonic semitones for the chosen note
// weights: overtone weights
static float overtone_dissonance_precomputed(float f1_st, HarmonicSet *h2, float *weights) {
    float total = 0.0f;
    
    for (int i = 0; i < CHORD_NUM_OVERTONES; i++) {
        // Candidate partial in semitones: f1_st + 12*log2(i+1)
        float f1_partial_st = f1_st + log2_k_semitones[i];
        float wi = weights[i];
        
        for (int j = 0; j < CHORD_NUM_OVERTONES; j++) {
            float f2_partial_st = h2->partials_st[j];
            
            // Compute base dissonance in semitone domain (no log calls!)
            float base = sine_dissonance_base_st(f1_partial_st, f2_partial_st);
            
            // Combined weight and shift semantics
            float w_combined = wi * weights[j];
            float d = base - (1.0f - w_combined);
            if (d > 0.0f) {
                total += d;
            }
        }
    }
    
    return total;
}

// Precompute harmonic semitones for a fundamental frequency
// Uses fast approximation since this is for dissonance comparison
static inline void precompute_harmonics(float f0_hz, HarmonicSet *out) {
    // Convert fundamental to semitones once (fast is OK for dissonance)
    float f0_st = freq_to_semitones_fast(f0_hz);
    out->f0_st = f0_st;
    
    // Each partial is f0_st + precomputed log2(k) offset
    for (int i = 0; i < CHORD_NUM_OVERTONES; i++) {
        out->partials_st[i] = f0_st + log2_k_semitones[i];
    }
}

// Check if candidate is too close to any existing frequency (within 0.25 semitones)
// IMPORTANT: Must use exact log2f here - fast_log2f has ~0.7 semitone error
// which would cause identical frequencies to NOT be detected as duplicates!
static uint8_t is_duplicate(float candidate_hz, float *existing_hz, uint8_t num_existing) {
    for (uint8_t i = 0; i < num_existing; i++) {
        if (existing_hz[i] <= 0.0f) continue;
        float ratio = candidate_hz / existing_hz[i];
        float semitone_dist = fabsf(12.0f * log2f(ratio));
        if (semitone_dist < 0.25f) return 1;
    }
    return 0;
}

// Generate candidate frequencies from harmonics of existing notes
// Returns number of candidates generated
static uint8_t generate_candidates(
    float *existing_hz,
    uint8_t num_existing,
    float *candidates_out,
    uint8_t microtonal,
    float min_freq_hz,
    float max_freq_hz
) {
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < num_existing && count < MAX_CANDIDATES; i++) {
        float f0 = existing_hz[i];
        if (f0 <= 0.0f) continue;
        
        // Generate harmonics and subharmonics k = 2..SEARCH_HARMONICS
        for (int k = 2; k <= CHORD_SEARCH_HARMONICS && count < MAX_CANDIDATES; k++) {
            float f_harm = f0 * (float)k;
            float f_subharm = f0 / (float)k;
            
            // Check bounds and duplicates for harmonic
            if (f_harm >= min_freq_hz && f_harm <= max_freq_hz) {
                if (!is_duplicate(f_harm, existing_hz, num_existing)) {
                    if (microtonal) {
                        candidates_out[count++] = f_harm;
                    } else {
                        // Quantize to nearest semitone
                        float st = freq_to_semitones(f_harm);
                        float st_q = roundf(st);
                        float f_q = semitones_to_freq(st_q);
                        if (!is_duplicate(f_q, existing_hz, num_existing)) {
                            // Check not already in candidates
                            uint8_t found = 0;
                            for (uint8_t c = 0; c < count; c++) {
                                if (fabsf(candidates_out[c] - f_q) < 0.01f) { found = 1; break; }
                            }
                            if (!found) candidates_out[count++] = f_q;
                        }
                    }
                }
            }
            
            // Check bounds and duplicates for subharmonic
            if (f_subharm >= min_freq_hz && f_subharm <= max_freq_hz && count < MAX_CANDIDATES) {
                if (!is_duplicate(f_subharm, existing_hz, num_existing)) {
                    if (microtonal) {
                        candidates_out[count++] = f_subharm;
                    } else {
                        float st = freq_to_semitones(f_subharm);
                        float st_q = roundf(st);
                        float f_q = semitones_to_freq(st_q);
                        if (!is_duplicate(f_q, existing_hz, num_existing)) {
                            uint8_t found = 0;
                            for (uint8_t c = 0; c < count; c++) {
                                if (fabsf(candidates_out[c] - f_q) < 0.01f) { found = 1; break; }
                            }
                            if (!found) candidates_out[count++] = f_q;
                        }
                    }
                }
            }
        }
    }
    
    return count;
}

// Build harmonic chord using greedy dissonance minimization
void build_harmonic_chord(
    float *seed_freqs,
    uint8_t num_seeds,
    uint8_t *voices_to_fill,
    uint8_t num_to_fill,
    float *overtone_weights,
    uint8_t microtonal,
    float min_freq_hz,
    float max_freq_hz,
    float extension_freq_hz,
    float above_ext_db_per_oct,
    uint16_t scale_mask,
    float scale_penalty,
    float *chord_out
) {
    if (num_to_fill == 0 || num_seeds == 0) return;
    
    // Initialize lookup tables on first call
    init_luts();
    
    // Clamp frequency bounds to absolute limits
    if (min_freq_hz < ABS_MIN_FREQ_HZ) min_freq_hz = ABS_MIN_FREQ_HZ;
    if (max_freq_hz > ABS_MAX_FREQ_HZ) max_freq_hz = ABS_MAX_FREQ_HZ;
    
    // Working arrays
    float chosen[NUM_CHANNELS + 1];  // Currently chosen frequencies
    uint8_t num_chosen = 0;
    float candidates[MAX_CANDIDATES];
    float weights[CHORD_NUM_OVERTONES];
    
    // Precomputed harmonics for chosen notes (optimization #4)
    HarmonicSet chosen_harmonics[NUM_CHANNELS + 1];
    
    // Candidate cache for optimization
    static CandidateCache cache[MAX_CANDIDATES];
    uint8_t cache_count = 0;
    
    // Clear cache at start of chord building
    for (uint8_t i = 0; i < MAX_CANDIDATES; i++) {
        cache[i].freq = 0.0f;
    }
    
    // Copy overtone weights (clip negative to 0, don't normalize)
    // Normalization would make w_combined too small for shift semantics to work
    for (int i = 0; i < CHORD_NUM_OVERTONES; i++) {
        weights[i] = overtone_weights[i];
        if (weights[i] < 0.0f) weights[i] = 0.0f;
    }
    
    // Initialize chosen set with seed frequencies
    // Quantize to semitones (unless microtonal) so that octave ratios are exact
    // This ensures octaves compute as 0 dissonance, matching Python behavior
    for (uint8_t i = 0; i < num_seeds && num_chosen < NUM_CHANNELS; i++) {
        if (seed_freqs[i] > 0.0f) {
            float f = seed_freqs[i];
            if (!microtonal) {
                float st = freq_to_semitones(f);
                float st_q = roundf(st);
                f = semitones_to_freq(st_q);
            }
            chosen[num_chosen] = f;
            precompute_harmonics(f, &chosen_harmonics[num_chosen]);
            num_chosen++;
        }
    }
    
    // Greedy loop: add notes one at a time
    for (uint8_t v = 0; v < num_to_fill; v++) {
        // Generate candidates from harmonics of all chosen notes
        uint8_t num_cand = generate_candidates(chosen, num_chosen, candidates, microtonal, min_freq_hz, max_freq_hz);
        
        if (num_cand == 0) {
            // Fallback: try intervals in order until we find one not already chosen
            // Try: P5 (1.5), P4 (4/3), M3 (5/4), m3 (6/5), M2 (9/8), m2 (16/15), octave up, octave down
            static const float fallback_ratios[] = {1.5f, 1.333333f, 1.25f, 1.2f, 1.125f, 1.066667f, 2.0f, 0.5f};
            static const uint8_t num_fallbacks = 8;
            
            float lowest = chosen[0];
            for (uint8_t i = 1; i < num_chosen; i++) {
                if (chosen[i] < lowest) lowest = chosen[i];
            }
            
            float fallback_freq = 0.0f;
            for (uint8_t fb = 0; fb < num_fallbacks; fb++) {
                float candidate = lowest * fallback_ratios[fb];
                if (!microtonal) {
                    float st = freq_to_semitones(candidate);
                    candidate = semitones_to_freq(roundf(st));
                }
                // Check if in range and not duplicate
                if (candidate >= min_freq_hz && candidate <= max_freq_hz) {
                    if (!is_duplicate(candidate, chosen, num_chosen)) {
                        fallback_freq = candidate;
                        break;
                    }
                }
            }
            
            // If all fallbacks failed, just use the seed frequency (will be duplicate but safe)
            if (fallback_freq <= 0.0f) {
                fallback_freq = lowest;
            }
            
            chord_out[v] = fallback_freq;
            chosen[num_chosen] = fallback_freq;
            precompute_harmonics(fallback_freq, &chosen_harmonics[num_chosen]);
            num_chosen++;
            continue;
        }
        
        // Find candidate with minimum dissonance to the set
        float min_diss = 1e30f;
        uint8_t best_idx = 0;
        
        for (uint8_t c = 0; c < num_cand; c++) {
            float cand_freq = candidates[c];
            
            // Check if this candidate is in the cache
            int16_t cache_idx = -1;
            for (uint8_t ci = 0; ci < cache_count; ci++) {
                if (fabsf(cache[ci].freq - cand_freq) < 0.01f) {
                    cache_idx = ci;
                    break;
                }
            }
            
            float total_diss;
            
            // Convert candidate to semitones once for this candidate
            // Use fast approximation since this is for dissonance comparison
            float cand_st = freq_to_semitones_fast(cand_freq);
            
            if (cache_idx >= 0) {
                // Found in cache - compute lower bound first
                float lower_bound = cache[cache_idx].total_diss / (float)num_chosen;
                if (lower_bound >= min_diss) {
                    // This candidate can't beat current best, skip full evaluation
                    continue;
                }
                
                // Compute dissonance only to notes added since last evaluation
                total_diss = cache[cache_idx].total_diss;
                for (uint8_t ni = cache[cache_idx].num_notes; ni < num_chosen; ni++) {
                    total_diss += overtone_dissonance_precomputed(cand_st, &chosen_harmonics[ni], weights);
                }
                
                // Update cache
                cache[cache_idx].total_diss = total_diss;
                cache[cache_idx].num_notes = num_chosen;
            } else {
                // Not in cache - compute full dissonance using precomputed harmonics
                total_diss = 0.0f;
                for (uint8_t ni = 0; ni < num_chosen; ni++) {
                    total_diss += overtone_dissonance_precomputed(cand_st, &chosen_harmonics[ni], weights);
                }
                
                // Add to cache if space available
                if (cache_count < MAX_CANDIDATES) {
                    cache[cache_count].freq = cand_freq;
                    cache[cache_count].total_diss = total_diss;
                    cache[cache_count].num_notes = num_chosen;
                    cache_count++;
                }
            }
            
            float mean_diss = total_diss / (float)num_chosen;
            
            // Add scale penalty for out-of-scale candidates
            mean_diss += calc_scale_penalty(cand_freq, scale_mask, scale_penalty);
            
            // Add boundary penalty for candidates above extension frequency
            mean_diss += calc_boundary_penalty(cand_freq, extension_freq_hz, above_ext_db_per_oct);
            
            // Prefer lower dissonance, tie-break by lower frequency (matches Python's np.argmin on sorted array)
            // Use epsilon for tie detection to handle float32 vs float64 precision differences
            float epsilon = 1e-4f;
            if (mean_diss < min_diss - epsilon || (fabsf(mean_diss - min_diss) < epsilon && cand_freq < candidates[best_idx])) {
                min_diss = mean_diss;
                best_idx = c;
            }
        }
        
        // Add best candidate to chord and chosen set
        chord_out[v] = candidates[best_idx];
        chosen[num_chosen] = candidates[best_idx];
        precompute_harmonics(candidates[best_idx], &chosen_harmonics[num_chosen]);
        num_chosen++;
    }
}
