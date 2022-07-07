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
	for (auto &axis : axes) {
		axis->calculateAxisEffects(isActive());
	}

	if(!isActive()){
	 return;
	}

	float forceX = 0;
	float forceY = 0;
	float forceVector = 0;
	uint8_t axisCount = (uint8_t)axes.size();
	bool validY = axisCount > 1;
#if MAX_AXIS == 3
	int32_t forceZ = 0;
	bool validZ = axisCount > 2;
#endif

	for (uint8_t i = 0; i < MAX_EFFECTS; i++)
	{
		FFB_Effect *effect = &effects[i];

		// Effect activated and not infinite (0 or 0xffff)
		if (effect->state != EFFECT_STATE_INACTIVE && effect->duration != FFB_EFFECT_DURATION_INFINITE && effect->duration != 0){
			// Start delay not yet reached
			if(HAL_GetTick() < effect->startTime){
				continue;
			}
			// If effect has expired make inactive
			if (HAL_GetTick() > effect->startTime + effect->duration)
			{
				effect->state = EFFECT_STATE_INACTIVE;
			}
		}

		// Filter out inactive effects
		if (effect->state == EFFECT_STATE_INACTIVE)
		{
			continue;
		}


		//if (effect->conditionsCount == 0) {
		forceVector = calcNonConditionEffectForce(effect);
		//}

		uint8_t directionEnableMask = this->directionEnableMask ? this->directionEnableMask : DIRECTION_ENABLE(axisCount);

		if (effect->enableAxis & directionEnableMask || (effect->enableAxis & X_AXIS_ENABLE))
		{
			forceX += calcComponentForce(effect, forceVector, axes, 0);
			forceX = clip<float, float>(forceX, (float)-0x7fff, (float)0x7fff); // Clip
		}
		if (validY && (effect->enableAxis & directionEnableMask || (effect->enableAxis & Y_AXIS_ENABLE)))
		{
			forceY += calcComponentForce(effect, forceVector, axes, 1);
			forceY = clip<float, float>(forceY, (float)-0x7fff, (float)0x7fff); // Clip
		}

	}

	axes[0]->setEffectTorque((int32_t)forceX);
	if (validY)
	{
		axes[1]->setEffectTorque((int32_t)forceY);
	}
}

/**
 * Calculates forces from a non conditional effect
 * Periodic and constant effects
 */
float EffectsCalculator::calcNonConditionEffectForce(FFB_Effect *effect) {
	float force_vector = 0;
	float magnitude = effect->magnitude;

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
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		int32_t duration = effect->duration;
		force_vector = (float)(effect->startLevel + ((int32_t)elapsed_time * (effect->endLevel - effect->startLevel)) / duration);
		break;
	}

	case FFB_EFFECT_SQUARE:
	{
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		float force = ((elapsed_time + effect->phase) % ((uint32_t)effect->period + 2)) < (uint32_t)(effect->period + 2) / 2 ? -magnitude : magnitude;
		force_vector = force + effect->offset;
		break;
	}

	case FFB_EFFECT_TRIANGLE:
	{
		float force = 0;
		float offset = effect->offset;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		float periodF = (float)period;

		float maxMagnitude = offset + magnitude;
		float minMagnitude = offset - magnitude;
		uint32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		float remainder = (float)(timeTemp % period);
		float slope = ((maxMagnitude - minMagnitude) * 2.0f) / periodF;
		if (remainder > (periodF / 2.0f))
			force = slope * (periodF - remainder);
		else
			force = slope * remainder;
		force += minMagnitude;
		force_vector = force;
		break;
	}

	case FFB_EFFECT_SAWTOOTHUP:
	{
		float offset = effect->offset;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		float periodF = effect->period;

		float maxMagnitude = offset + magnitude;
		float minMagnitude = offset - magnitude;
		int32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		float remainder = (float)(timeTemp % period);
		float slope = (maxMagnitude - minMagnitude) / periodF;
		force_vector = minMagnitude + slope * (periodF - remainder);
		break;
	}

	case FFB_EFFECT_SAWTOOTHDOWN:
	{
		float offset = effect->offset;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		float periodF = effect->period;

		float maxMagnitude = offset + magnitude;
		float minMagnitude = offset - magnitude;
		int32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		float remainder = (float)(timeTemp % period);
		float slope = (maxMagnitude - minMagnitude) / periodF;
		force_vector = minMagnitude + slope * (remainder); // reverse time
		break;
	}

	case FFB_EFFECT_SINE:
	{
		float t = (float)(HAL_GetTick() - effect->startTime);
		float freq = 1.0f / (float)(std::max<uint16_t>(effect->period, 2));
		float phase = (float)effect->phase / 35999.0f; //degrees
		float sine = sinf(2.0f * (float)M_PI * (t * freq + phase)) * magnitude;
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

float EffectsCalculator::calcComponentForce(FFB_Effect *effect, float forceVector, std::vector<std::unique_ptr<Axis>> &axes, uint8_t axis)
{
	float result_torque = 0.0f;
	uint16_t direction;
	uint8_t con_idx = 0; // condition block index

	metric_t *metrics = axes[axis]->getMetrics();
	uint8_t axisCount = (uint8_t)axes.size();
	float scaleSpeed = 40.0f;//axes[axis]->getSpeedScalerNormalized(); // TODO decide if scalers are useful or not
	float scaleAccel = 40.0f;//axes[axis]->getAccelScalerNormalized();
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

	//bool useForceDirectionForConditionEffect = (effect->enableAxis == DIRECTION_ENABLE && axisCount > 1 && effect->conditionsCount == 1);
	bool rotateConditionForce = (axisCount > 1); // && effect->conditionsCount < axisCount
	float angle = ((float)direction * (2.0f * (float)M_PI) / 36000.0f);
	float angle_ratio = axis == 0 ? sinf(angle) : -1.0f * cosf(angle);
	angle_ratio = rotateConditionForce ? angle_ratio : 1.0f;

	switch (effect->type)
	{
	case FFB_EFFECT_CONSTANT:
	{
		// Optional filtering to reduce spikes
		if (cfFilter_f < calcfrequency / 2 && cfFilter_f != 0 )
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
		float pos = (float)metrics->pos;
		result_torque -= calcConditionEffectForce(effect, pos, gain.spring, con_idx, spring_scaler, angle_ratio);
		break;
	}



	case FFB_EFFECT_FRICTION:
	{

		float speed = metrics->speed * scaleSpeed;
		result_torque -= effect->filter[con_idx]->process(calcConditionEffectForce(effect, speed, gain.friction, con_idx, friction_scaler, angle_ratio));

		break;
	}
	case FFB_EFFECT_DAMPER:
	{

		float speed = metrics->speed * scaleSpeed;
		result_torque -= effect->filter[con_idx]->process(calcConditionEffectForce(effect, speed, gain.damper, con_idx, damper_scaler, angle_ratio));

		break;
	}

	case FFB_EFFECT_INERTIA:
	{
		float accel = metrics->accel* scaleAccel;
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
float EffectsCalculator::calcConditionEffectForce(FFB_Effect *effect, float  metric, uint8_t gain,
										 uint8_t idx, float scale, float angle_ratio)
{
	float offset = effect->conditions[idx].cpOffset;
	float deadBand = effect->conditions[idx].deadBand;
	float force = 0;
	float gainfactor = (float)(gain+1) / 256.0f;

	// Effect is only active outside deadband + offset
	if (abs(metric - offset) > deadBand){
		float coefficient = effect->conditions[idx].negativeCoefficient;
		if(metric > offset){
			coefficient = effect->conditions[idx].positiveCoefficient;
		}
		coefficient /= (float)0x7fff; // rescale the coefficient of effect

		// remove offset/deadband from metric to compute force
		metric = metric - (offset + (deadBand * (metric < offset ? -1.0f : 1.0f)) );

		force = clip<float, float>((coefficient * gainfactor * scale * metric),
										(float)-effect->conditions[idx].negativeSaturation,
										(float)effect->conditions[idx].positiveSaturation);
	}


	return force * angle_ratio;
}

/**
 * Modulates the magnitude of an effect based on time and attack/fade levels
 * During attack time the strength changes from the initial attack level to the normal magnitude which is sustained
 * until the fade time where the strength changes to the fade level until the stop time of the effect.
 * Infinite effects can't have an envelope and return the normal magnitude.
 */
float EffectsCalculator::getEnvelopeMagnitude(FFB_Effect *effect)
{
	if(effect->duration == FFB_EFFECT_DURATION_INFINITE || effect->duration == 0){
		return effect->magnitude; // Effect is infinite. envelope is invalid
	}
	int32_t scaler = abs(effect->magnitude);
	uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
	if (elapsed_time < effect->attackTime && effect->attackTime != 0)
	{
		scaler = (scaler - effect->attackLevel) * elapsed_time;
		scaler /= (int32_t)effect->attackTime;
		scaler += effect->attackLevel;
	}
	if (elapsed_time > (effect->duration - effect->fadeTime) && effect->fadeTime != 0)
	{
		scaler = (scaler - effect->fadeLevel) * (effect->duration - elapsed_time); // Reversed
		scaler /= (int32_t)effect->fadeTime;
		scaler += effect->fadeLevel;
	}
	scaler = signbit(effect->magnitude) ? -scaler : scaler; // Follow original sign of magnitude because envelope has no sign (important for constant force)
	return (float)scaler;
}

void EffectsCalculator::setFilters(FFB_Effect *effect){

	std::function<void(std::unique_ptr<Biquad> &)> fnptr = [=](std::unique_ptr<Biquad> &filter){};

	switch (effect->type)
	{
	case FFB_EFFECT_DAMPER:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, (float)damper_f / (float)calcfrequency, damper_q, (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, (float)damper_f / (float)calcfrequency, damper_q, (float)0.0);
		};
		break;
	case FFB_EFFECT_FRICTION:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, (float)friction_f / (float)calcfrequency, friction_q, (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, (float)friction_f / (float)calcfrequency, friction_q, (float)0.0);
		};
		break;
	case FFB_EFFECT_INERTIA:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, (float)inertia_f / (float)calcfrequency, inertia_q, (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, (float)inertia_f / (float)calcfrequency, inertia_q, (float)0.0);
		};
		break;
	case FFB_EFFECT_CONSTANT:
		fnptr = [=](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, (float)cfFilter_f / (float)calcfrequency, cfFilter_qfloatScaler * (cfFilter_q+1), (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, (float)cfFilter_f / (float)calcfrequency, cfFilter_qfloatScaler * (cfFilter_q+1), (float)0.0);
		};
		break;
	}


	for (int i=0; i<MAX_AXIS; i++) {
		fnptr(effect->filter[i]);
	}
}


void EffectsCalculator::setGain(uint8_t gain)
{
	global_gain = (float)gain / 255.0f;
}

uint8_t EffectsCalculator::getGain() { return (uint8_t)(global_gain * 255.0f); }

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

	if(freq == 0){
		freq = calcfrequency / 2;
	}
	cfFilter_f = clip<uint32_t, uint32_t>(freq, 1, (calcfrequency / 2));
	//float f = (float)cfFilter_f / (float)calcfrequency;

	for (uint8_t i = 0; i < MAX_EFFECTS; i++)
	{
		if (effects[i].type == FFB_EFFECT_CONSTANT)
		{
			setFilters(&effects[i]);
			//for(uint8_t ax = 0;ax<MAX_AXIS;ax++){

//				effects[i].filter[ax]->setFc(f);
//				effects[i].filter[ax]->setQ(cfFilter_qfloatScaler * (cfFilter_q+1));
		//	}

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

