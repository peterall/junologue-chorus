#include "usermodfx.h"

#include "delayline.hpp"
#include "simplelfo.hpp"
#include "biquad.hpp"

#include <algorithm>

static constexpr float samplerate = 48000.f;

typedef struct {
  float min;
  float max;
} delay_t;

typedef enum {
  MODE_I,
  MODE_II,
  MODE_I_II
} mode_t;

static constexpr delay_t delay_times[] = {
  { .min = 0.00154, .max = 0.00515 }, 
  { .min = 0.00151, .max = 0.00540 }
};

static constexpr float rates[] = {
  0.513f, 0.863f
};

static constexpr float _0db = 1.f, _3db = 0.707f, _infdb = 0.f;
static constexpr f32pair_t gains[] = {
  { _0db, _infdb },
  { _infdb, _0db },
  { _3db, _3db }
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

static constexpr size_t delay_size = 
  nextpow2_u32_constexpr(static_cast<uint32_t>(std::max(delay_times[0].max, delay_times[1].max) * samplerate) + 1);


static dsp::DelayLine delay;
static /* __sdram */ float delay_ram[delay_size];

static dsp::SimpleLFO lfo_1, lfo_2;

static dsp::BiQuad pre_lpf, post_lpf_l, post_lpf_r;

static mode_t mode = MODE_I;
static float wet_dry = .5f;
static const float drive = 1.45f;

// the chips are effectively sampling at about 70kHz
// a 12dB low-pass filter is used before the signal is sampled

void MODFX_INIT(uint32_t platform, uint32_t api)
{
  delay.setMemory(delay_ram, delay_size);  
  lfo_1.reset();
  lfo_1.setF0(rates[0], 1.f / samplerate);
  lfo_2.reset();
  lfo_2.setF0(rates[1], 1.f / samplerate);
  pre_lpf.flush();
  pre_lpf.mCoeffs.setFOLP(fx_tanpif(7237.f / samplerate));
  post_lpf_l.flush();
  post_lpf_l.mCoeffs.setFOLP(fx_tanpif(10644.f / samplerate));
  post_lpf_r.flush();
  post_lpf_r.mCoeffs.setFOLP(fx_tanpif(10644.f / samplerate));
}

inline float readDelay(const delay_t &d, const float p) {
  return delay.readFrac((d.min + ((d.max - d.min) * p)) * samplerate);
}

inline f32pair_t readDelays(const float p) {
  return {
    readDelay(delay_times[0], p),
    readDelay(delay_times[1], 1.f - p)
  };
}

void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn,  float *sub_yn,
                   uint32_t frames)
{
  const f32pair_t __restrict__ *input = reinterpret_cast<const f32pair_t*>(main_xn);
  f32pair_t __restrict__ *output = reinterpret_cast<f32pair_t*>(main_yn);

  for(uint32_t i=0;i<frames;i++) {
    delay.write(pre_lpf.process_so((input[i].a + input[i].b) * _3db));

    lfo_1.cycle();
    lfo_2.cycle();

    const auto s = f32pair_add(
      f32pair_mulscal(readDelays(lfo_1.triangle_uni()), gains[mode].a),
      f32pair_mulscal(readDelays(lfo_2.triangle_uni()), gains[mode].b));

    output[i].a = SoftClip(Crossfade(input[i].a, post_lpf_l.process_fo(s.a), wet_dry) * drive);
    output[i].b = SoftClip(Crossfade(input[i].b, post_lpf_r.process_fo(s.b), wet_dry) * drive);
  }

  // ignore the sub on prologue for now
  buf_cpy_f32(sub_xn, sub_yn, frames);
}

void MODFX_PARAM(uint8_t index, int32_t value)
{
  // normalize to 0..1
  const float valf = q31_to_f32(value);
  switch (index) {
  case k_user_modfx_param_time:
    mode = static_cast<mode_t>(valf * 2.9999f);
    break;
  case k_user_modfx_param_depth:
    wet_dry = valf;
    break;
  default:
    break;
  }
}

