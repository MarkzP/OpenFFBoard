/*
 * Filters.cpp
 *
 *  Created on: Feb 13, 2020
 *      Author: Yannick
 */

#include "Filters.h"

#include <math.h>


Biquad::Biquad(){
	z1 = z2 = 0.0;
}
Biquad::Biquad(BiquadType type, float Fc, float Q, float peakGainDB) {
    setBiquad(type, Fc, Q, peakGainDB);
}

Biquad::~Biquad() {
}

void Biquad::setFc(float Fc) {
	Fc = clip<float,float>(Fc,0,0.5);
    this->Fc = Fc;
    calcBiquad();
}

void Biquad::setQ(float Q) {
    this->Q = Q;
    calcBiquad();
}

float Biquad::process(float in) {
	float out = in * a0 + z1;
    z1 = in * a1 + z2 - b1 * out;
    z2 = in * a2 - b2 * out;
    return out;
}

void Biquad::setBiquad(BiquadType type, float Fc, float Q, float peakGainDB) {
	Fc = clip<float,float>(Fc,0,0.5);
    this->type = type;
    this->Q = Q;
    this->Fc = Fc;
    this->peakGain = peakGainDB;
    calcBiquad();
}

/*
 * Updates parameters and resets the biquad filter
 */
void Biquad::calcBiquad(void) {
	z1 = 0.0;
	z2 = 0.0;
    float norm;
    float V = powf(10, fabsf(peakGain) / 20.0f);
    float K = tanf((float)M_PI * Fc);
    switch (this->type) {
        case BiquadType::lowpass:
            norm = 1.0f / (1.0f + K / Q + K * K);
            a0 = K * K * norm;
            a1 = 2.0f * a0;
            a2 = a0;
            b1 = 2.0f * (K * K - 1.0f) * norm;
            b2 = (1.0f - K / Q + K * K) * norm;
            break;

        case BiquadType::highpass:
            norm = 1.0f / (1.0f + K / Q + K * K);
            a0 = 1.0f * norm;
            a1 = -2.0f * a0;
            a2 = a0;
            b1 = 2.0f * (K * K - 1.0f) * norm;
            b2 = (1.0f - K / Q + K * K) * norm;
            break;

        case BiquadType::bandpass:
            norm = 1.0f / (1.0f + K / Q + K * K);
            a0 = K / Q * norm;
            a1 = 0;
            a2 = -a0;
            b1 = 2.0f * (K * K - 1.0f) * norm;
            b2 = (1.0f - K / Q + K * K) * norm;
            break;

        case BiquadType::notch:
            norm = 1.0f / (1.0f + K / Q + K * K);
            a0 = (1.0f + K * K) * norm;
            a1 = 2.0f * (K * K - 1.0f) * norm;
            a2 = a0;
            b1 = a1;
            b2 = (1.0f - K / Q + K * K) * norm;
            break;

        case BiquadType::peak:
            if (peakGain >= 0) {    // boost
                norm = 1.0f / (1.0f + 1.0f/Q * K + K * K);
                a0 = (1.0f + V/Q * K + K * K) * norm;
                a1 = 2.0f * (K * K - 1) * norm;
                a2 = (1.0f - V/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1.0f - 1.0f/Q * K + K * K) * norm;
            }
            else {    // cut
                norm = 1.0f / (1.0f + V/Q * K + K * K);
                a0 = (1.0f + 1.0f/Q * K + K * K) * norm;
                a1 = 2.0f * (K * K - 1.0f) * norm;
                a2 = (1.0f - 1.0f/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1.0f - V/Q * K + K * K) * norm;
            }
            break;
        case BiquadType::lowshelf:
            if (peakGain >= 0) {    // boost
                norm = 1.0f / (1.0f + sqrtf(2.0f) * K + K * K);
                a0 = (1.0f + sqrtf(2.0f*V) * K + V * K * K) * norm;
                a1 = 2.0f * (V * K * K - 1.0f) * norm;
                a2 = (1.0f - sqrtf(2.0f*V) * K + V * K * K) * norm;
                b1 = 2.0f * (K * K - 1.0f) * norm;
                b2 = (1.0f - sqrtf(2.0f) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1.0f / (1.0f + sqrtf(2.0f*V) * K + V * K * K);
                a0 = (1.0f + sqrtf(2.0f) * K + K * K) * norm;
                a1 = 2.0f * (K * K - 1.0f) * norm;
                a2 = (1.0f - sqrtf(2.0f) * K + K * K) * norm;
                b1 = 2.0f * (V * K * K - 1) * norm;
                b2 = (1.0f - sqrtf(2.0f*V) * K + V * K * K) * norm;
            }
            break;
        case BiquadType::highshelf:
            if (peakGain >= 0) {    // boost
                norm = 1.0f / (1.0f + sqrtf(2.0f) * K + K * K);
                a0 = (V + sqrtf(2.0f*V) * K + K * K) * norm;
                a1 = 2.0f * (K * K - V) * norm;
                a2 = (V - sqrtf(2.0f*V) * K + K * K) * norm;
                b1 = 2.0f * (K * K - 1.0f) * norm;
                b2 = (1.0f - sqrtf(2.0f) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1.0f / (V + sqrtf(2.0f*V) * K + K * K);
                a0 = (1.0f + sqrtf(2.0f) * K + K * K) * norm;
                a1 = 2.0f * (K * K - 1.0f) * norm;
                a2 = (1.0f - sqrtf(2.0f) * K + K * K) * norm;
                b1 = 2.0f * (K * K - V) * norm;
                b2 = (V - sqrtf(2.0f*V) * K + K * K) * norm;
            }
            break;
    }

    return;
}
