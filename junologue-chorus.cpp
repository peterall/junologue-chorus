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

static constexpr f32pair_t delay_gains[] = {
  { _0db, _infdb },
  { _3db, _3db },
  { _infdb, _0db }
};

static constexpr size_t mode_count = sizeof(delay_gains) / sizeof(f32pair_t);

static constexpr delay_t delay_times[] = {
  { .min = 0.00154, .max = 0.00515 }, 
  { .min = 0.00151, .max = 0.00533 }
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

static dsp::DelayLine main_delay, sub_delay;;
static float main_delay_ram[delay_size], sub_delay_ram[delay_size];

static dsp::SimpleLFO lfo_1, lfo_2;

static dsp::BiQuad main_pre_lpf, sub_pre_lpf, 
  main_post_lpf_l, main_post_lpf_r, 
  sub_post_lpf_l, sub_post_lpf_r;

static uint32_t mode = 0;
static float mode_frac = 0.f;
static float dry_gain = _3db, wet_gain = _3db;

// the chips are effectively sampling at about 70kHz
// a 12dB low-pass filter is used before the signal is sampled

void MODFX_INIT(uint32_t platform, uint32_t api)
{
  main_delay.setMemory(main_delay_ram, delay_size);  
  main_delay.clear();
  sub_delay.setMemory(sub_delay_ram, delay_size);  
  sub_delay.clear();

  main_pre_lpf.flush();
  main_pre_lpf.mCoeffs.setFOLP(fx_tanpif(7237.f / samplerate));
  sub_pre_lpf.flush();
  sub_pre_lpf.mCoeffs = main_pre_lpf.mCoeffs;

  lfo_1.reset();
  lfo_1.setF0(rates[0], 1.f / samplerate);
  lfo_2.reset();
  lfo_2.setF0(rates[1], 1.f / samplerate);

  main_post_lpf_l.flush();
  main_post_lpf_l.mCoeffs.setFOLP(fx_tanpif(10644.f / samplerate));
  main_post_lpf_r.flush();
  main_post_lpf_r.mCoeffs = main_post_lpf_l.mCoeffs;

  sub_post_lpf_l.flush();
  sub_post_lpf_l.mCoeffs = main_post_lpf_l.mCoeffs;
  sub_post_lpf_r.flush();
  sub_post_lpf_r.mCoeffs = main_post_lpf_l.mCoeffs;
}

inline float readDelay(dsp::DelayLine &delay, const delay_t &t, const float p) {
  return delay.readFrac((t.min + ((t.max - t.min) * p)) * samplerate);
}

inline f32pair_t readDelays(dsp::DelayLine &delay, const float p) {
  return {
    readDelay(delay, delay_times[0], p),
    readDelay(delay, delay_times[1], 1.f - p)
  };
}

inline f32pair_t interpolateGain(const float fr, const uint32_t x, const uint32_t y) {
    return { 
      linintf(fr, delay_gains[x].a, delay_gains[y].a),
      linintf(fr, delay_gains[x].b, delay_gains[y].b)
    };
}

inline f32pair_t delayGains() {
  if(mode_frac < .1f && mode > 0) {
    return interpolateGain(.5f + (mode_frac * 5.f), mode - 1, mode);
  } else if(mode_frac > .9f && mode < mode_count - 1) {
    return interpolateGain((mode_frac - .9f) * 5.f, mode, mode + 1);
  } else {
    return delay_gains[mode];
  }
}

inline f32pair_t f32pair_process_fo(dsp::BiQuad &fa, dsp::BiQuad &fb, const f32pair_t &s) {
  return { fa.process_fo(s.a), fb.process_fo(s.b) };
}

inline float f32pair_reduce(const f32pair_t &p) {
  return (p.a + p.b); //* _3db;
}

void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn,  float *sub_yn,
                   uint32_t frames)
{
  const f32pair_t __restrict__ *main_input = reinterpret_cast<const f32pair_t*>(main_xn);
  f32pair_t __restrict__ *main_output = reinterpret_cast<f32pair_t*>(main_yn);

  const f32pair_t __restrict__ *sub_input = reinterpret_cast<const f32pair_t*>(sub_xn);
  f32pair_t __restrict__ *sub_output = reinterpret_cast<f32pair_t*>(sub_yn);

  const f32pair_t *main_output_end = main_output + frames;

  const auto delay_gain = delayGains();

  while(main_output != main_output_end) {
    main_delay.write(main_pre_lpf.process_fo(SoftLimit(f32pair_reduce(*main_input))));
    sub_delay.write(sub_pre_lpf.process_fo(SoftLimit(f32pair_reduce(*sub_input))));

    lfo_1.cycle();
    lfo_2.cycle();

    const float lfo_val1 = lfo_1.triangle_uni(), lfo_val2 = lfo_2.triangle_uni();

    const auto main = f32pair_add(
      f32pair_mulscal(readDelays(main_delay, lfo_val1), delay_gain.a),
      f32pair_mulscal(readDelays(main_delay, lfo_val2), delay_gain.b));

    *main_output++ = f32pair_add(
      f32pair_mulscal(*main_input++, dry_gain), 
      f32pair_mulscal(f32pair_process_fo(main_post_lpf_l, main_post_lpf_r, main), wet_gain));

    const auto sub = f32pair_add(
      f32pair_mulscal(readDelays(sub_delay, lfo_val1), delay_gain.a),
      f32pair_mulscal(readDelays(sub_delay, lfo_val2), delay_gain.b));

    *sub_output++ = f32pair_add(
      f32pair_mulscal(*sub_input++, dry_gain), 
      f32pair_mulscal(f32pair_process_fo(sub_post_lpf_l, sub_post_lpf_r, sub), wet_gain));
  }
}

void MODFX_PARAM(uint8_t index, int32_t value)
{
  const float valf = q31_to_f32(value);
  switch (index) {
  case k_user_modfx_param_time:
    {
      const float v = valf * (static_cast<float>(mode_count) - 0.0001f);
      mode = static_cast<uint32_t>(v);
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

