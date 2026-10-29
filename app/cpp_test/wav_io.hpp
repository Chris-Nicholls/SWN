/*
 * wav_io.hpp — tiny mono 16-bit PCM WAV writer.  Header-only.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace wav {

inline bool write_mono_f32(const std::string& path,
                           const std::vector<float>& samples,
                           uint32_t sample_rate = 48000)
{
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    const uint32_t n          = (uint32_t)samples.size();
    const uint16_t channels   = 1;
    const uint16_t bits       = 16;
    const uint32_t byte_rate  = sample_rate * channels * bits / 8;
    const uint16_t block_align= channels * bits / 8;
    const uint32_t data_bytes = n * block_align;

    auto wu32 = [&](uint32_t v){
        uint8_t b[4]={(uint8_t)v,(uint8_t)(v>>8),(uint8_t)(v>>16),(uint8_t)(v>>24)};
        std::fwrite(b,1,4,f);
    };
    auto wu16 = [&](uint16_t v){
        uint8_t b[2]={(uint8_t)v,(uint8_t)(v>>8)};
        std::fwrite(b,1,2,f);
    };

    std::fwrite("RIFF",1,4,f);
    wu32(36 + data_bytes);
    std::fwrite("WAVE",1,4,f);
    std::fwrite("fmt ",1,4,f);
    wu32(16);                  /* PCM fmt chunk size */
    wu16(1);                   /* PCM */
    wu16(channels);
    wu32(sample_rate);
    wu32(byte_rate);
    wu16(block_align);
    wu16(bits);
    std::fwrite("data",1,4,f);
    wu32(data_bytes);

    for (float s : samples) {
        float v = s * 32767.0f;
        if (v >  32767.0f) v =  32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        int16_t i = (int16_t)v;
        uint8_t b[2]={(uint8_t)i,(uint8_t)(i>>8)};
        std::fwrite(b,1,2,f);
    }

    std::fclose(f);
    return true;
}

} /* namespace wav */
