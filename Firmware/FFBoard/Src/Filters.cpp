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
Biquad::Biquad(BiquadType type, double Fc, double Q, double peakGainDB) {
    setBiquad(type, Fc, Q, peakGainDB);
}

Biquad::~Biquad() {
}

/**
 * Sets the frequency
 * Calculate as Fc = f/samplerate
 * Must be lower than 0.5
 */
void Biquad::setFc(double Fc) {
	Fc = clip<double,double>(Fc,0,0.5);
	if (Fc == this->Fc) {
		z1 = 0.0;
		z2 = 0.0;
		return;
	}
    this->Fc = Fc;
    calcBiquad();
}

/**
 * Changes Q value and recalculaes filter
 */
void Biquad::setQ(double Q) {
    if (Q == this->Q) {
    	z1 = 0.0;
    	z2 = 0.0;
    	return;
    }
	this->Q = Q;
    calcBiquad();
}

/**
 * Calculates one step of the filter and returns the output
 */
double Biquad::process(double in) {
	double out = in * a0 + z1;
    z1 = in * a1 + z2 - b1 * out;
    z2 = in * a2 - b2 * out;
    return out;
}

void Biquad::setBiquad(BiquadType type, double Fc, double Q, double peakGainDB) {
	Fc = clip<double,double>(Fc,0,0.5);
	if (type == this->type && Q == this->Q && Fc == this->Fc && peakGainDB == this->peakGain) {
		z1 = 0.0;
		z2 = 0.0;
		return;
	}
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
    double norm;
    double V;
    double K = tan(M_PI * Fc);
    switch (this->type) {
        case BiquadType::lowpass:
            norm = 1.0 / (1.0 + K / Q + K * K);
            a0 = K * K * norm;
            a1 = 2.0 * a0;
            a2 = a0;
            b1 = 2.0 * (K * K - 1.0) * norm;
            b2 = (1.0 - K / Q + K * K) * norm;
            break;

        case BiquadType::highpass:
            norm = 1.0 / (1.0 + K / Q + K * K);
            a0 = 1.0 * norm;
            a1 = -2.0 * a0;
            a2 = a0;
            b1 = 2.0 * (K * K - 1.0) * norm;
            b2 = (1.0 - K / Q + K * K) * norm;
            break;

        case BiquadType::bandpass:
            norm = 1.0 / (1.0 + K / Q + K * K);
            a0 = K / Q * norm;
            a1 = 0;
            a2 = -a0;
            b1 = 2.0 * (K * K - 1.0) * norm;
            b2 = (1.0 - K / Q + K * K) * norm;
            break;

        case BiquadType::notch:
            norm = 1.0 / (1.0 + K / Q + K * K);
            a0 = (1.0 + K * K) * norm;
            a1 = 2.0 * (K * K - 1.0) * norm;
            a2 = a0;
            b1 = a1;
            b2 = (1.0 - K / Q + K * K) * norm;
            break;

        case BiquadType::peak:
        	V = pow(10.0, abs(peakGain) / 20.0);
        	if (peakGain >= 0) {    // boost
                norm = 1.0 / (1.0 + 1.0/Q * K + K * K);
                a0 = (1.0 + V/Q * K + K * K) * norm;
                a1 = 2.0 * (K * K - 1) * norm;
                a2 = (1.0 - V/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1.0 - 1.0/Q * K + K * K) * norm;
            }
            else {    // cut
                norm = 1.0 / (1.0 + V/Q * K + K * K);
                a0 = (1.0 + 1.0/Q * K + K * K) * norm;
                a1 = 2.0 * (K * K - 1.0) * norm;
                a2 = (1.0 - 1.0/Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1.0 - V/Q * K + K * K) * norm;
            }
            break;
        case BiquadType::lowshelf:
        	V = pow(10.0, abs(peakGain) / 20.0);
            if (peakGain >= 0) {    // boost
                norm = 1.0 / (1.0 + sqrt(2.0) * K + K * K);
                a0 = (1.0 + sqrt(2.0*V) * K + V * K * K) * norm;
                a1 = 2.0 * (V * K * K - 1.0) * norm;
                a2 = (1.0 - sqrt(2.0*V) * K + V * K * K) * norm;
                b1 = 2.0 * (K * K - 1.0) * norm;
                b2 = (1.0 - sqrt(2.0) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1.0 / (1.0 + sqrt(2.0*V) * K + V * K * K);
                a0 = (1.0 + sqrt(2.0) * K + K * K) * norm;
                a1 = 2.0 * (K * K - 1.0) * norm;
                a2 = (1.0 - sqrt(2.0) * K + K * K) * norm;
                b1 = 2.0 * (V * K * K - 1) * norm;
                b2 = (1.0 - sqrt(2.0*V) * K + V * K * K) * norm;
            }
            break;
        case BiquadType::highshelf:
        	V = pow(10.0, abs(peakGain) / 20.0);
            if (peakGain >= 0) {    // boost
                norm = 1.0 / (1.0 + sqrt(2.0) * K + K * K);
                a0 = (V + sqrt(2.0*V) * K + K * K) * norm;
                a1 = 2.0 * (K * K - V) * norm;
                a2 = (V - sqrt(2.0*V) * K + K * K) * norm;
                b1 = 2.0 * (K * K - 1.0) * norm;
                b2 = (1.0 - sqrt(2.0) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1.0 / (V + sqrt(2.0*V) * K + K * K);
                a0 = (1.0 + sqrt(2.0) * K + K * K) * norm;
                a1 = 2.0 * (K * K - 1.0) * norm;
                a2 = (1.0 - sqrt(2.0) * K + K * K) * norm;
                b1 = 2.0 * (K * K - V) * norm;
                b2 = (V - sqrt(2.0*V) * K + K * K) * norm;
            }
            break;
        case BiquadType::bypass:
        		a0 = 1.0;
        		a1 = a2 = b1 = b2 = 0.0;
        	break;
    }

    return;
}
