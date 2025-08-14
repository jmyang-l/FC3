#ifndef __LOW_PASS_FILTER_H__
#define __LOW_PASS_FILTER_H__


#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float _cutoff_freq;
        float _a1, _a2;
        float _b0;
        float _delay_element_1, _delay_element_2;
    } low_pass_filter_t;

    extern void lpf_set_cutoff_frequency(low_pass_filter_t *p, float sample_freq, float cutoff_freq);
    extern float lpf_allpy(low_pass_filter_t *p, float sample);
    extern float lpf_reset(low_pass_filter_t *p, float sample);

#ifdef __cplusplus
}
#endif
#endif
