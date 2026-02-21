#ifndef ARM_MATH_MOCK_H
#define ARM_MATH_MOCK_H

#include <math.h>
#include <stdint.h>

// Mock some CMSIS types/functions if needed
typedef float float32_t;

typedef struct {
  uint32_t numStages;
  float32_t *pState;
  float32_t *pCoeffs;
} arm_biquad_casd_df1_inst_f32;

#ifdef __cplusplus
extern "C" {
#endif
void arm_biquad_cascade_df1_f32(const arm_biquad_casd_df1_inst_f32 *S, float32_t *pSrc, float32_t *pDst, uint32_t blockSize);
#ifdef __cplusplus
}
#endif

#endif
