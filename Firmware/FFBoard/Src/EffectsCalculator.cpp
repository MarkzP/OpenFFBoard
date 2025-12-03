/*
 * EffectsCalculator.cpp
 *
 *  Created on: 27.01.21
 *      Author: Jon Lidgard
 *      Yannick Richter
 */

#include <stdint.h>
#include "math.h"
#include "EffectsCalculator.h"
#include "Axis.h"

#define X_AXIS_ENABLE 1
#define Y_AXIS_ENABLE 2
#define Z_AXIS_ENABLE 4
#define DIRECTION_ENABLE(AXES) (1 << AXES)

#define EFFECT_STATE_INACTIVE 0

ClassIdentifier EffectsCalculator::info = {
		  .name = "Effects" ,
		  .id	= CLSID_EFFECTSCALC,
		  .visibility = ClassVisibility::hidden
};
const ClassIdentifier EffectsCalculator::getInfo(){
	return info;
}


EffectsCalculator::EffectsCalculator() : CommandHandler("fx", CLSID_EFFECTSCALC)
{
	restoreFlash();

	CommandHandler::registerCommands();
	registerCommand("filterCfFreq", EffectsCalculator_commands::ffbfiltercf, "Constant force filter frequency", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("filterCfQ", EffectsCalculator_commands::ffbfiltercf_q, "Constant force filter Q-factor", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("spring", EffectsCalculator_commands::spring, "Spring gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("friction", EffectsCalculator_commands::friction, "Friction gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("damper", EffectsCalculator_commands::damper, "Damper gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("inertia", EffectsCalculator_commands::inertia, "Inertia gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("effects", EffectsCalculator_commands::effects, "List effects. set 0 to reset", CMDFLAG_GET | CMDFLAG_SET  | CMDFLAG_STR_ONLY);
}

EffectsCalculator::~EffectsCalculator()
{

}


bool EffectsCalculator::isActive()
{
	return effects_active;
}
void EffectsCalculator::setActive(bool active)
{
	effects_active = active;
}

/**
 * Sets the mask where the direction enable bit is in the effect
 */
void EffectsCalculator::setDirectionEnableMask(uint8_t mask){
	this->directionEnableMask = mask;
}

/*
If the metric is less than CP Offset - Dead Band, then the resulting force is given by the following formula:
		force = Negative Coefficient * (q - (CP Offset – Dead Band))
Similarly, if the metric is greater than CP Offset + Dead Band, then the resulting force is given by the
following formula:
		force = Positive Coefficient * (q - (CP Offset + Dead Band))
A spring condition uses axis position as the metric.
A damper condition uses axis velocity as the metric.
An inertia condition uses axis acceleration as the metric.

If the number of Condition report blocks is equal to the number of axes for the effect, then the first report
block applies to the first axis, the second applies to the second axis, and so on. For example, a two-axis
spring condition with CP Offset set to zero in both Condition report blocks would have the same effect as
the joystick self-centering spring. When a condition is defined for each axis in this way, the effect must
not be rotated.

If there is a single Condition report block for an effect with more than one axis, then the direction along
which the parameters of the Condition report block are in effect is determined by the direction parameters
passed in the Direction field of the Effect report block. For example, a friction condition rotated 45
degrees (in polar coordinates) would resist joystick motion in the northeast-southwest direction but would
have no effect on joystick motion in the northwest-southeast direction.
 */

/*
 * Calculates the resulting torque for FFB effects
 * Takes current position input scaled from -0x7fff to 0x7fff
 * Outputs a torque value from -0x7fff to 0x7fff (not yet clipped)
 */
void EffectsCalculator::calculateEffects(std::vector<std::unique_ptr<Axis>> &axes)
{
	double forceX = 0;
	double forceY = 0;
	uint8_t axisCount = (uint8_t)axes.size();
	bool validY = axisCount > 1;
#if MAX_AXIS == 3
	int32_t forceZ = 0;
	bool validZ = axisCount > 2;
#endif

	axes[0]->calculateAxisEffects(isActive());
	if (validY)
	{
		axes[1]->calculateAxisEffects(isActive());
	}

	if(!isActive()){
	 return;
	}

	for (uint8_t i = 0; i < MAX_EFFECTS; i++)
	{
		FFB_Effect *effect = &effects[i];

		// Effect activated and not infinite (0 or 0xffff)
		if (effect->state != EFFECT_STATE_INACTIVE && effect->duration != FFB_EFFECT_DURATION_INFINITE && effect->duration != 0){
			// Start delay not yet reached
			if(effect->startDelay != 0 && HAL_GetTick() < effect->startTime){
				continue;
			}
			// If effect has expired make inactive
			if (HAL_GetTick() - effect->startTime > effect->duration)
			{
				effect->state = EFFECT_STATE_INACTIVE;
			}
		}

		// Filter out inactive effects
		if (effect->state == EFFECT_STATE_INACTIVE)
		{
			continue;
		}

		double forceVector = calcNonConditionEffectForce(effect);

		uint8_t directionEnableMask = this->directionEnableMask ? this->directionEnableMask : DIRECTION_ENABLE(axisCount);

		if (effect->enableAxis & directionEnableMask || (effect->enableAxis & X_AXIS_ENABLE))
		{
			forceX += calcComponentForce(effect, forceVector, axes, 0);
		}
		if (validY && (effect->enableAxis & directionEnableMask || (effect->enableAxis & Y_AXIS_ENABLE)))
		{
			forceY += calcComponentForce(effect, forceVector, axes, 1);
		}
	}

	axes[0]->setEffectTorque(forceX);
	if (validY)
	{
		axes[1]->setEffectTorque(forceY);
	}
}

/**
 * Calculates forces from a non conditional effect
 * Periodic and constant effects
 */
double EffectsCalculator::calcNonConditionEffectForce(FFB_Effect *effect) {
	double force_vector = 0;
	double magnitude = effect->magnitude;

	// If using an envelope modulate the magnitude based on time
	if(effect->useEnvelope){
		magnitude = getEnvelopeMagnitude(effect);
	}
	switch (effect->type){

	case FFB_EFFECT_CONSTANT:
	{ // Constant force is just the force
		force_vector = magnitude;
		break;
	}

	case FFB_EFFECT_RAMP:
	{
		double elapsed_time = (double)(HAL_GetTick() - effect->startTime);
		double duration = (double)effect->duration;
		force_vector = effect->startLevel + (elapsed_time * (effect->endLevel - effect->startLevel)) / duration;
		break;
	}

	case FFB_EFFECT_SQUARE:
	{
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		double force = ((elapsed_time + effect->phase) % ((uint32_t)effect->period + 2)) < (uint32_t)(effect->period + 2) / 2 ? -magnitude : magnitude;
		force_vector = force + effect->offset;
		break;
	}

	case FFB_EFFECT_TRIANGLE:
	{
		double force = 0;
		double offset = effect->offset;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		double periodF = (double)period;

		double maxMagnitude = offset + magnitude;
		double minMagnitude = offset - magnitude;
		uint32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		double remainder = (double)(timeTemp % period);
		double slope = ((maxMagnitude - minMagnitude) * 2.0) / periodF;
		if (remainder > (periodF / 2.0))
			force = slope * (periodF - remainder);
		else
			force = slope * remainder;
		force += minMagnitude;
		force_vector = force;
		break;
	}

	case FFB_EFFECT_SAWTOOTHUP:
	{
		double offset = effect->offset;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		double periodF = effect->period;

		double maxMagnitude = offset + magnitude;
		double minMagnitude = offset - magnitude;
		int32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		double remainder = (double)(timeTemp % period);
		double slope = (maxMagnitude - minMagnitude) / periodF;
		force_vector = minMagnitude + slope * (periodF - remainder);
		break;
	}

	case FFB_EFFECT_SAWTOOTHDOWN:
	{
		double offset = effect->offset;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		double periodF = effect->period;

		double maxMagnitude = offset + magnitude;
		double minMagnitude = offset - magnitude;
		int32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		double remainder = (double)(timeTemp % period);
		double slope = (maxMagnitude - minMagnitude) / periodF;
		force_vector = minMagnitude + slope * (remainder); // reverse time
		break;
	}

	case FFB_EFFECT_SINE:
	{
		double t = (double)(HAL_GetTick() - effect->startTime);
		double freq = 1.0 / (double)(std::max<uint16_t>(effect->period, 2));
		double phase = (double)effect->phase / 35999.0; //degrees
		double sine = sin(2.0 * M_PI * (t * freq + phase)) * magnitude;
		force_vector = effect->offset + sine;
		break;
	}
	default:
		return 0;
		break;
	}

	return force_vector * effect->gain;
}



/*
 * If the number of Condition report blocks is equal to the number of axes for the effect, then the first report
block applies to the first axis, the second applies to the second axis, and so on. For example, a two-axis
spring condition with CP Offset set to zero in both Condition report blocks would have the same effect as
the joystick self-centering spring. When a condition is defined for each axis in this way, the effect must
not be rotated.

If there is a single Condition report block for an effect with more than one axis, then the direction along
which the parameters of the Condition report block are in effect is determined by the direction parameters
passed in the Direction field of the Effect report block. For example, a friction condition rotated 45
degrees (in polar coordinates) would resist joystick motion in the northeast-southwest direction but would
have no effect on joystick motion in the northwest-southeast direction.
 */

double EffectsCalculator::calcComponentForce(FFB_Effect *effect, double forceVector, std::vector<std::unique_ptr<Axis>> &axes, uint8_t axis)
{
	double result_torque = 0.0;
	uint16_t direction;
	uint8_t con_idx = 0; // condition block index

	metric_t *metrics = axes[axis]->getMetrics();
	uint8_t axisCount = (uint8_t)axes.size();
	double scaleSpeed = 40.0;
	double scaleAccel = 40.0;
	uint8_t directionEnableMask = this->directionEnableMask ? this->directionEnableMask : DIRECTION_ENABLE(axisCount);
	if (effect->enableAxis & directionEnableMask)
	{
		direction = effect->directionX;
//		if (effect->conditionsCount > 1)
//		{
			con_idx = axis;
//		}
	}
	else
	{
		direction = axis == 0 ? effect->directionX : effect->directionY;
		con_idx = axis;
	}

	double angle_ratio = 1.0;
	bool rotateConditionForce = (axisCount > 1); // && effect->conditionsCount < axisCount
	if (rotateConditionForce)
	{
		double angle = ((double)direction * (2.0 * M_PI) / 36000.0);
		angle_ratio = axis == 0 ? sin(angle) : -1.0 * cos(angle);
	}

	switch (effect->type)
	{
	case FFB_EFFECT_CONSTANT:
	{
		// Optional filtering to reduce spikes
		if (cfFilter_f > 0.0)
		{
			forceVector = effect->filter[con_idx]->process(forceVector);
		}
	}
	case FFB_EFFECT_RAMP:
	case FFB_EFFECT_SQUARE:
	case FFB_EFFECT_TRIANGLE:
	case FFB_EFFECT_SAWTOOTHUP:
	case FFB_EFFECT_SAWTOOTHDOWN:
	case FFB_EFFECT_SINE:
	{
		result_torque = -forceVector * angle_ratio;
		break;
	}

	case FFB_EFFECT_SPRING:
	{
		double pos = (double)metrics->pos;
		result_torque -= calcConditionEffectForce(effect, pos, gain.spring, con_idx, spring_scaler, angle_ratio);
		break;
	}



	case FFB_EFFECT_FRICTION:
	{

		double speed = metrics->speed * scaleSpeed;

		double deadBand = effect->conditions[con_idx].deadBand;
		double force = 0.0;

		double sign = speed < 0.0 ? -1.0 : 1.0;
		double coeff = speed < 0.0 ? (double)effect->conditions[con_idx].negativeCoefficient : (double)effect->conditions[con_idx].positiveCoefficient;

		double speedRampupCeil = 1000.0 + (coeff * 0.25);

		// Effect is only active outside deadband + offset
		if (abs(speed) > deadBand){

			// remove offset/deadband from metric to compute force
			speed -= deadBand * (speed < 0.0 ? -1.0 : 1.0);

			double rampupFactor = 1.0;
			if (abs(speed) < speedRampupCeil) {								// if speed in the range to rampup we apply a sinus curbe to ramup

				double phaseRad = M_PI * ((abs(speed) / speedRampupCeil) - 0.5);// we start to compute the normalized angle (speed / normalizedSpeed@5%) and translate it of -1/2PI to translate sin on 1/2 periode
				rampupFactor = (1.0 + sin(phaseRad)) * 0.5;						// sin value is -1..1 range, we translate it to 0..2 and we scale it by 2
			}

			force = coeff * rampupFactor * sign;
			force *= 0.25;

			force = clip<double, double>(force, (double)-effect->conditions[con_idx].negativeSaturation, (double)effect->conditions[con_idx].positiveSaturation);

			force *= (double)(gain.friction + 1);
			force /= 256.0;
			force *= angle_ratio;
		}

		result_torque -= effect->filter[con_idx]->process(force);

		break;
	}
	case FFB_EFFECT_DAMPER:
	{
		effect->conditions[con_idx].cpOffset = 0.0;
		double speed = metrics->speed * scaleSpeed;
		result_torque -= effect->filter[con_idx]->process(calcConditionEffectForce(effect, speed, gain.damper, con_idx, damper_scaler, angle_ratio));

		break;
	}

	case FFB_EFFECT_INERTIA:
	{
		effect->conditions[con_idx].cpOffset = 0.0;
		double accel = metrics->accel* scaleAccel;
		result_torque -= effect->filter[con_idx]->process(calcConditionEffectForce(effect, accel, gain.inertia, con_idx, inertia_scaler, angle_ratio)); // Bump *60 the inertia feedback

		break;
	}

	default:
		// Unsupported effect
		break;
	}

	return result_torque * global_gain; // Apply global gain
}

/**
 * Calculates a conditional effect
 * Takes care of deadband and offsets and scalers
 * Gain of 255 = 1x. Prescale with scale factor
 */
double EffectsCalculator::calcConditionEffectForce(FFB_Effect *effect, double  metric, uint8_t gain,
										 uint8_t idx, double scale, double angle_ratio)
{
	double offset = effect->conditions[idx].cpOffset;
	double deadBand = effect->conditions[idx].deadBand;
	double force = 0;
	double gainfactor = (double)(gain+1) / 256.0;

	// Effect is only active outside deadband + offset
	if (abs(metric - offset) > deadBand){
		double coefficient = effect->conditions[idx].negativeCoefficient;
		if(metric > offset){
			coefficient = effect->conditions[idx].positiveCoefficient;
		}
		coefficient /= (double)0x7fff; // rescale the coefficient of effect

		// remove offset/deadband from metric to compute force
		metric = metric - (offset + (deadBand * (metric < offset ? -1.0 : 1.0)) );

		force = clip<double, double>((coefficient * gainfactor * scale * metric),
										-effect->conditions[idx].negativeSaturation,
										effect->conditions[idx].positiveSaturation);
	}


	return force * angle_ratio;
}

/**
 * Modulates the magnitude of an effect based on time and attack/fade levels
 * During attack time the strength changes from the initial attack level to the normal magnitude which is sustained
 * until the fade time where the strength changes to the fade level until the stop time of the effect.
 * Infinite effects can't have an envelope and return the normal magnitude.
 */
double EffectsCalculator::getEnvelopeMagnitude(FFB_Effect *effect)
{
	if(effect->duration == FFB_EFFECT_DURATION_INFINITE || effect->duration == 0){
		return effect->magnitude; // Effect is infinite. envelope is invalid
	}
	double duration = (double)effect->duration;
	double scaler = abs(effect->magnitude);
	double elapsed_time = (double)(HAL_GetTick() - effect->startTime);
	if (elapsed_time < effect->attackTime && effect->attackTime > 0.0)
	{
		scaler = (scaler - effect->attackLevel) * elapsed_time;
		scaler /= effect->attackTime;
		scaler += effect->attackLevel;
	}
	if (elapsed_time > (duration - effect->fadeTime) && effect->fadeTime > 0)
	{
		scaler = (scaler - effect->fadeLevel) * (duration - elapsed_time); // Reversed
		scaler /= effect->fadeTime;
		scaler += effect->fadeLevel;
	}
	scaler = signbit(effect->magnitude) ? -scaler : scaler; // Follow original sign of magnitude because envelope has no sign (important for constant force)
	return scaler;
}

void EffectsCalculator::setFilters(FFB_Effect *effect){

	std::function<void(std::unique_ptr<Biquad> &)> fnptr = [=](std::unique_ptr<Biquad> &filter){};

	switch (effect->type)
	{
	case FFB_EFFECT_DAMPER:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass_1p1z, damper_f / calcfrequency, damper_q, 0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass_1p1z, damper_f / calcfrequency, damper_q, 0.0);
		};
		break;
	case FFB_EFFECT_FRICTION:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass_1p1z, friction_f / calcfrequency, friction_q, 0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass_1p1z, friction_f / calcfrequency, friction_q, 0.0);
		};
		break;
	case FFB_EFFECT_INERTIA:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass_1p1z, inertia_f / calcfrequency, inertia_q, 0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass_1p1z, inertia_f / calcfrequency, inertia_q, 0.0);
		};
		break;
	case FFB_EFFECT_CONSTANT:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			BiquadType ftype = cfFilter_f == 0 ? BiquadType::bypass : cfFilter_q == 0 ? BiquadType::lowpass_1p1z : BiquadType::lowpass;
			if (filter != nullptr)
				filter->setBiquad(ftype, (double)cfFilter_f / calcfrequency, cfFilter_qdoubleScaler * (double)(cfFilter_q+1), 0.0);
			else
				filter = std::make_unique<Biquad>(ftype, (double)cfFilter_f / calcfrequency, cfFilter_qdoubleScaler * (double)(cfFilter_q+1), 0.0);
		};
		break;
	}


	for (int i=0; i<MAX_AXIS; i++) {
		fnptr(effect->filter[i]);
	}
}


void EffectsCalculator::setGain(uint8_t gain)
{
	global_gain = (double)gain / 255.0;
}

uint8_t EffectsCalculator::getGain() { return (uint8_t)(global_gain * 255.0); }

void EffectsCalculator::setEffectsArray(FFB_Effect *pEffects)
{
	effects = pEffects;
}



/*
 * Read parameters from flash and restore settings
 */
void EffectsCalculator::restoreFlash()
{
	uint16_t filter;
	if (Flash_Read(ADR_CF_FILTER, &filter))
	{
		this->cfFilter_f = filter & 0x1FF;
		this->cfFilter_q = (filter >> 9) & 0x7F;
	}
	setCfFilter(this->cfFilter_f,this->cfFilter_q);

	uint16_t effects = 0;
	if(Flash_Read(ADR_AXIS_EFFECTS1, &effects)){
		gain.friction = (effects >> 8) & 0xff;
		gain.inertia = (effects & 0xff);
	}
	if(Flash_Read(ADR_AXIS_EFFECTS2, &effects)){
		gain.damper = (effects >> 8) & 0xff;
		gain.spring = (effects & 0xff);
	}

}

// Saves parameters to flash
void EffectsCalculator::saveFlash()
{
	uint16_t cffilter = (cfFilter_f & 0x1FF) | ((cfFilter_q & 0x7F) << 9);
	Flash_Write(ADR_CF_FILTER, cffilter);
	uint16_t effects = gain.inertia | (gain.friction << 8);
	Flash_Write(ADR_AXIS_EFFECTS1, effects);
	effects = gain.spring | (gain.damper << 8);
	Flash_Write(ADR_AXIS_EFFECTS2, effects);

}

void EffectsCalculator::setCfFilter(uint32_t freq,uint8_t q)
{
	this->cfFilter_q = clip<uint8_t, uint8_t>(q,0,127);

	cfFilter_f = clip<uint32_t, uint32_t>(freq, 0, (uint32_t)nyquist);

	for (uint8_t i = 0; i < MAX_EFFECTS; i++)
	{
		if (effects[i].type == FFB_EFFECT_CONSTANT)
		{
			setFilters(&effects[i]);
		}
	}
}

void EffectsCalculator::logEffectType(uint8_t type){
	if(type > 0 && type < 32){
		effects_used |= 1<<(type-1);
	}
}

/**
 * Prints a list of effects that were active at some point
 * Does not reset when an effect is deactivated
 */
std::string EffectsCalculator::listEffectsUsed(){
	std::string effects_list = "";
	static const char *effects[12] = {"Constant,","Ramp,","Square,","Sine,","Triangle,","Sawtooth Up,","Sawtooth Down,","Spring,","Damper,","Inertia,","Friction,","Custom,"};

	if(effects_used == 0){
		return "None";
	}

	for (int i=0;i < 12; i++) {
		if((effects_used >> i) & 1) {
			effects_list += effects[i];
		}
	}
	effects_list.pop_back();
	return effects_list;
}


CommandStatus EffectsCalculator::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
	switch(static_cast<EffectsCalculator_commands>(cmd.cmdId)){

	case EffectsCalculator_commands::ffbfiltercf:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(cfFilter_f);
		}
		else if (cmd.type == CMDtype::set)
		{
			setCfFilter(cmd.val,this->cfFilter_q);
		}
		break;

	case EffectsCalculator_commands::ffbfiltercf_q:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(cfFilter_q);
		}
		else if (cmd.type == CMDtype::set)
		{
			setCfFilter(this->cfFilter_f,cmd.val);
		}

		break;

	case EffectsCalculator_commands::effects:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(listEffectsUsed());
		}
		else if (cmd.type == CMDtype::set && cmd.val == 0)
		{
			effects_used = 0;
		}
		break;
	case EffectsCalculator_commands::spring:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(this->spring_scaler));
		}else
			return handleGetSet(cmd, replies, this->gain.spring);
		break;
	case EffectsCalculator_commands::friction:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(2));
		}else
			return handleGetSet(cmd, replies, this->gain.friction);
		break;
	case EffectsCalculator_commands::damper:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(2));
		}else
			return handleGetSet(cmd, replies, this->gain.damper);
		break;
	case EffectsCalculator_commands::inertia:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(2));
		}else
			return handleGetSet(cmd, replies, this->gain.inertia);
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}
	return CommandStatus::OK;
}

