/*
 * Filters.h
 *
 *  Created on: Feb 13, 2020
 *      Author: Yannick
 */

#ifndef FILTERS_H_
#define FILTERS_H_
#include "cppmain.h"

#ifdef __cplusplus

enum class BiquadType : uint8_t {
    lowpass = 0,
    highpass,
    bandpass,
    notch,
    peak,
    lowshelf,
    highshelf,
	bypass
};

class Biquad{
public:
	Biquad();
    Biquad(BiquadType type, double Fc, double Q, double peakGainDB);
    ~Biquad();
    double process(double in);
    void setBiquad(BiquadType type, double Fc, double Q, double peakGain);
    void setFc(double Fc); //frequency
    void setQ(double Q);
    void calcBiquad(void);

protected:

    BiquadType type;
    double a0, a1, a2, b1, b2;
    double Fc, Q, peakGain;
    double z1, z2;
};


#endif

#endif /* FILTERS_H_ */
