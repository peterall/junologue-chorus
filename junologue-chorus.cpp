#include "usermodfx.h"

#include "delayline.hpp"
#include "simplelfo.hpp"
#include "biquad.hpp"

#include <algorithm>

static constexpr float samplerate = 48000.f;
static constexpr float _0db = 1.f, _3db = M_1_SQRT2, _infdb = 0.f;

typedef struct {
  float min;
  float max;
} delay_t;

typedef enum {
  MODE_I,
  MODE_I_II,
  MODE_II,
  NUMBER_OF_MODES
} mode_t;

static constexpr f32pair_t delay_gains[] = {
  { _0db, _infdb },
  { _3db, _3db },
  { _infdb, _0db }
};

static constexpr delay_t delay_times[] = {
  { .min = 0.00154, .max = 0.00515 }, 
  { .min = 0.00151, .max = 0.00540 }
};

static constexpr float rates[] = {
  0.513f, 0.863f
};

constexpr uint32_t nextpow2_u32_constexpr(uint32_t x) {
  x--;
  x |= x>>1; x |= x>>2;
  x |= x>>4; x |= x>>8;
  x |= x>>16;
  return ++x;
}

inline float fastsqrt(float number)
{
	float x2 = number * 0.5f;
	float y  = number;
	long i  = * ( long * ) &y;
	i  = 0x5f3759df - ( i >> 1 );
	y  = * ( float * ) &i;
	y  = y * ( 1.5f - ( x2 * y * y ) );
	y  = y * ( 1.5f - ( x2 * y * y ) );

	return 1.f / fabsf(y);
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
static float mode_frac = 0.f;
static float dry_gain = _3db, wet_gain = _3db;

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

inline f32pair_t interpolateGain(const float fr, const mode_t x, const mode_t y) {
    return { 
      linintf(fr, delay_gains[x].a, delay_gains[y].a),
      linintf(fr, delay_gains[x].b, delay_gains[y].b)
    };
}

inline f32pair_t delayGains() {
  if(mode_frac < .1f && mode > 0) {
    return interpolateGain(.5f + (mode_frac * 5.f), static_cast<mode_t>(mode-1), mode);
  } else if(mode_frac > .9f && mode < NUMBER_OF_MODES - 1) {
    return interpolateGain((mode_frac - .9f) * 5.f, mode, static_cast<mode_t>(mode+1));
  } else {
    return delay_gains[mode];
  }
}

inline float EqualPowerCrossfade(const float a, const float b, const float fade) {
  return (a * fastsqrt(1.f - fade)) + (b * fastsqrt(fade));
}

void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn,  float *sub_yn,
                   uint32_t frames)
{
  const f32pair_t __restrict__ *input = reinterpret_cast<const f32pair_t*>(main_xn);
  f32pair_t __restrict__ *output = reinterpret_cast<f32pair_t*>(main_yn);

  const auto delay_gain = delayGains();

  for(uint32_t i=0;i<frames;i++) {
    delay.write(pre_lpf.process_so(SoftLimit((input[i].a + input[i].b) * _3db)));

    lfo_1.cycle();
    lfo_2.cycle();

    const auto s = f32pair_add(
      f32pair_mulscal(readDelays(lfo_1.triangle_uni()), delay_gain.a),
      f32pair_mulscal(readDelays(lfo_2.triangle_uni()), delay_gain.b));

    output[i].a = dry_gain * input[i].a + wet_gain * post_lpf_l.process_fo(s.a);
    output[i].b = dry_gain * input[i].b + wet_gain * post_lpf_r.process_fo(s.b);
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
    {
      const float v = valf * (static_cast<float>(NUMBER_OF_MODES) - 0.0001f);
      mode = static_cast<mode_t>(v);
      mode_frac = v - mode;
    }
    break;
  case k_user_modfx_param_depth:
    dry_gain = fastsqrt(1.f - valf);
    wet_gain = fastsqrt(valf);
    break;
  default:
    break;
  }
}

