/*
 * File: biquad.cpp
 *
 * Simple execution environment test using provided biquad filters.
 *
 * 
 * 
 * 2018 (c) Korg
 *
 */
#include "usermodfx.h"

#include "delayline.hpp"
#include "simplelfo.hpp"

static constexpr float samplerate =       48000.f;
static constexpr float bbd_samplerate =   samplerate;
static constexpr float samplerate_recip = 1.f/samplerate;

typedef struct {
  float rate;
  float min_delay;
  float max_delay;
  bool stereo;
} chorus_params_t;

static constexpr chorus_params_t params = {
  .rate = 0.513f,
  .min_delay = 0.00166f,
  .max_delay = 0.00535f,
  .stereo = true
};

constexpr uint32_t nextpow2_u32_constexpr(uint32_t x) {
  x--;
  x |= x>>1; x |= x>>2;
  x |= x>>4; x |= x>>8;
  x |= x>>16;
  return ++x;
}

inline float Crossfade(float a, float b, float fade) {
  return a + (b - a) * fade;
}

inline float SoftLimit(float x) {
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

inline float SoftClip(float x) {
  if (x < -3.0f) {
    return -1.0f;
  } else if (x > 3.0f) {
    return 1.0f;
  } else {
    return SoftLimit(x);
  }
}

static constexpr size_t delay_size = nextpow2_u32_constexpr(static_cast<uint32_t>(params.max_delay * bbd_samplerate) + 1);


static dsp::DelayLine delay;
static __sdram float delay_ram[delay_size];

static dsp::SimpleLFO lfo;

static float wet_dry = .5f;
static float drive = 1.45f;


// the chips are effectively sampling at about 70kHz
// a 12dB low-pass filter is used before the signal is sampled

void MODFX_INIT(uint32_t platform, uint32_t api)
{
  delay.setMemory(delay_ram, delay_size);  
  lfo.reset();
  lfo.setF0(params.rate, samplerate_recip);
}

void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn,  float *sub_yn,
                   uint32_t frames)
{
  const f32pair_t __restrict__ *input = reinterpret_cast<const f32pair_t*>(main_xn);
  f32pair_t __restrict__ *output = reinterpret_cast<f32pair_t*>(main_yn);

  for(uint32_t i=0;i<frames;i++) {
    delay.write(input[i].a + input[i].b);

    lfo.cycle();
    const auto lfo_sample = lfo.triangle_uni();

    const auto l = delay.readFrac((params.min_delay + ((params.max_delay - params.min_delay) * lfo_sample)) * bbd_samplerate);
    const auto r = delay.readFrac((params.min_delay + ((params.max_delay - params.min_delay) * (1 - lfo_sample))) * bbd_samplerate);

    output[i].a = SoftClip(Crossfade(input[i].a, l, wet_dry) * drive);
    output[i].b = SoftClip(Crossfade(input[i].b, r, wet_dry) * drive);
  }

  // ignore the sub on prologue
  buf_cpy_f32(sub_xn, sub_yn, frames);
}


void MODFX_PARAM(uint8_t index, int32_t value)
{
  // normalize to 0..1
  const float valf = q31_to_f32(value);
  switch (index) {
  case k_user_modfx_param_time:
    //drive = 1 + valf;
    break;
  case k_user_modfx_param_depth:
    wet_dry = valf;
    break;
  default:
    break;
  }
}

