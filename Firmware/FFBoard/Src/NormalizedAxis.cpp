/*
 * NormalizedAxis.h
 *
 *  Created on: 21.01.2021
 *      Author: Lidgard
 */

#include "NormalizedAxis.h"


ClassIdentifier NormalizedAxis::info = {
	.name = "Axis",
	.id = 1,
	.unique = 'X',
	.hidden = false};
//
//const ClassIdentifier NormalizedAxis::getInfo()
//{
//	return info;
//}


NormalizedAxis::NormalizedAxis(char axis) {
	this->axis = axis; // Axis are indexed from 1-X to 3-Z

	if(axis == 'X'){
		flashAddrs = NormalizedAxisFlashAddrs_t({ADR_AXIS1_ENDSTOP, ADR_AXIS1_POWER, ADR_AXIS1_DEGREES,ADR_AXIS1_EFFECTS1});
	}else if(axis == 'Y'){
		flashAddrs = NormalizedAxisFlashAddrs_t({ADR_AXIS2_ENDSTOP, ADR_AXIS2_POWER, ADR_AXIS2_DEGREES,ADR_AXIS2_EFFECTS1});
	}else if(axis == 'Z'){
		flashAddrs = NormalizedAxisFlashAddrs_t({ADR_AXIS3_ENDSTOP, ADR_AXIS3_POWER, ADR_AXIS3_DEGREES,ADR_AXIS3_EFFECTS1});
	}

//	restoreFlash();
}

NormalizedAxis::~NormalizedAxis() {
//
}

const ClassIdentifier NormalizedAxis::getInfo() {
	return ClassIdentifier {NormalizedAxis::info.name, NormalizedAxis::info.id, axis, NormalizedAxis::info.hidden};
}

metric_t* NormalizedAxis::getMetrics() {
	return &metric.current;
}

int32_t NormalizedAxis::getLastScaledEnc() {
	return  clip(metric.current.pos,-0x7fff,0x7fff);
}


/*
 * Read parameters from flash and restore settings
 */
void NormalizedAxis::restoreFlash(){
	uint16_t esval, power;
	if(Flash_Read(flashAddrs.endstop, &esval)) {
		setFxRatio(esval & 0xff);
		setEndstopGain((esval >> 8) & 0xff);
	}


	if(Flash_Read(flashAddrs.power, &power)){
		setPower(power);
	}
	uint16_t deg_t;
	if(Flash_Read(flashAddrs.degrees, &deg_t)){
		this->degreesOfRotation = deg_t & 0x7fff;
		this->invertAxis = (deg_t >> 15) & 0x1;
		setDegrees(degreesOfRotation);
	}


	uint16_t effects;
	if(Flash_Read(flashAddrs.effects1, &effects)){
		setIdleSpringStrength(effects & 0xff);
		setDamperStrength((effects >> 8) & 0xff);
	}
}


// Saves parameters to flash
void NormalizedAxis::saveFlash(){
	Flash_Write(flashAddrs.endstop, fx_ratio_i | (endstop_gain << 8));
	Flash_Write(flashAddrs.power, power);
	Flash_Write(flashAddrs.degrees, (degreesOfRotation & 0x7fff) | (invertAxis << 15));
	Flash_Write(flashAddrs.effects1, idlespringstrength | (damperIntensity << 8));
}


int32_t NormalizedAxis::updateIdleSpringForce() {
	return clip<int32_t,int32_t>((int32_t)(-metric.current.pos*idlespringscale),-idlespringclip,idlespringclip);
}
/*
 * Set the strength of the spring effect if FFB is disabled
 */
void NormalizedAxis::setIdleSpringStrength(uint8_t spring){
	idlespringstrength = spring;
	if(spring == 0){
		idle_center = false;
	}else{
		idle_center = true;
	}
	idlespringclip = clip<int32_t,int32_t>((int32_t)spring*50,0,10000);
	idlespringscale = 0.5f + ((float)spring * 0.01f);
}

void NormalizedAxis::setDamperStrength(uint8_t damper){
	if(damperIntensity == 0) // Was off before. reinit filter
		this->damperFilter.calcBiquad(); // Reset Filter

	this->damperIntensity = damper;
}

/*
 * Called before HID effects are calculated
 * Should calculate always on and idle effects specific to the axis like idlespring and friction
 */
void NormalizedAxis::calculateAxisEffects(bool ffb_on){
	axisEffectTorque = 0;

	if(!ffb_on){
		axisEffectTorque += updateIdleSpringForce();
	}

	// Always active damper
	if(damperIntensity != 0){
		float speedFiltered = damperFilter.process((float)metric.current.speed) * (float)damperIntensity * 1.5f;
		axisEffectTorque -= clip<float, int32_t>(speedFiltered, -damperClip, damperClip);
	}
}


void NormalizedAxis::setEndstopGain(uint8_t val) {
	endstop_gain = val;
	endstopScale = (float)val * 0.1f;
}

void NormalizedAxis::setFxRatio(uint8_t val) {
	fx_ratio_i = val;
	endstopDamperScale = (float)val * 0.4f;
}


void NormalizedAxis::resetMetrics(int32_t new_pos= 0) { // pos is scaledEnc
	metric.current.pos = new_pos;
	metric.previous.pos = metric.current.pos;
	metric.current.speed = 0;
	metric.current.accel = 0;
	metric.previous.speed = 0;
	metric.previous.torque = 0;
	metric.current.torque = 0;
}


void NormalizedAxis::updateMetrics(int32_t new_pos) { // pos is scaledEnc
	metric.current.pos = new_pos;
	metric.current.speed = metric.current.pos - metric.previous.pos;
	metric.current.accel = metric.current.speed - metric.previous.speed;
	metric.previous.pos = metric.current.pos;
	metric.previous.speed = metric.current.speed;
	metric.previous.torque = metric.current.torque;
	metric.current.torque = 0;
}

void NormalizedAxis::setPower(uint16_t val){
	power = val;
	updateTorqueScaler();
}

uint16_t NormalizedAxis::getPower(){
	return power;
}

void  NormalizedAxis::updateTorqueScaler() {
	torqueScaler = ((float)power / (float)0x7fff);
}

float NormalizedAxis::getTorqueScaler(){
	return torqueScaler;
}


int32_t NormalizedAxis::getTorque() { return metric.current.torque; }

bool NormalizedAxis::isInverted() {
	return invertAxis; // TODO store in flash
}

void NormalizedAxis::setEffectTorque(int32_t torque) {
	effectTorque = torque;
}

// pass in ptr to receive the sum of the effects + endstop torque
// return true if torque is clipping
bool NormalizedAxis::updateTorque(int32_t* totalTorque) {

	// Scale effect torque
	float combinedTorque = (float)(effectTorque + axisEffectTorque) * torqueScaler;

	float endstopTorque = 0.0f;
	int32_t endstopOvershoot = abs(metric.current.pos) - 0x7fff;
	if (endstopOvershoot > 0)
	{
		if (metric.current.pos < 0) endstopOvershoot = -endstopOvershoot;

		endstopTorque -= (float)endstopOvershoot * endstopScale;
		endstopTorque -= (float)metric.current.speed * endstopDamperScale;
	}

	endstopTorque = endstopFilter.process(endstopTorque);

	// Calculate total torque
	int32_t torque = 0;
	int32_t maxEffectTorque = abs(combinedTorque);
	int32_t maxEndstop = abs(endstopTorque);
	bool sameDir = (combinedTorque > 0 && endstopTorque > 0) || (combinedTorque < 0 && endstopTorque < 0);

	if (maxEffectTorque > 0 && maxEndstop > 0)
	{
		if (sameDir)
		{
			torque = (float)(maxEffectTorque > maxEndstop ? combinedTorque : endstopTorque);
		}
		else
		{
			torque = (float)endstopTorque;
		}
	}
	else
	{
		torque = (float)(combinedTorque + endstopTorque);
	}
	
	torque = (invertAxis) ? -torque : torque;
	metric.current.torque = torque;
	torque = clip<int32_t, int32_t>(torque, -power, power);

	bool torqueChanged = torque != metric.previous.torque;

	if (abs(torque) == power){
		pulseClipLed();
	}

	*totalTorque = torque;
	return (torqueChanged);
}


void NormalizedAxis::setDegrees(uint16_t degrees){

	degrees &= 0x7fff;
	if(degrees == 0){
		nextDegreesOfRotation = lastdegreesOfRotation;
	}else{
		lastdegreesOfRotation = degreesOfRotation;
		nextDegreesOfRotation = degrees;
	}

}


void NormalizedAxis::processHidCommand(HID_Custom_Data_t* data) {

	switch (data->cmd){
		case HID_CMD_FFB_ESGAIN:
			if(data->type == HidCmdType::write) {
				endstop_gain = data->data;
			}
			if(data->type == HidCmdType::request){
				data->data = endstop_gain;
			}
		break;

		case HID_CMD_FFB_FXRATIO:
			if(data->type == HidCmdType::write) {
				setFxRatio(data->data);
			}
			else if(data->type == HidCmdType::request){
				data->data = fx_ratio_i;
			}
		break;

		case HID_CMD_FFB_DEGREES:
			if (data->type == HidCmdType::write){


				setDegrees(data->data);

			}
			else if (data->type == HidCmdType::request){
				data->data = degreesOfRotation;
			}
		break;
		case HID_CMD_FFB_IDLESPRING:
			if(data->type == HidCmdType::write) {
				setIdleSpringStrength(data->data);
			}
			else if(data->type == HidCmdType::request){
				data->data = idlespringstrength;
			}
		break;
		
		default:

		break;
		}
	return;
}


ParseStatus NormalizedAxis::command(ParsedCommand *cmd, std::string *reply)
{
	ParseStatus flag = ParseStatus::OK;
	// ------------ General commands ----------------

	if (cmd->cmd == "fxratio")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += std::to_string(fx_ratio_i);
		}
		else if (cmd->type == CMDtype::set)
		{
			setFxRatio(cmd->val);
		}
	}
	else if (cmd->cmd == "power")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += std::to_string(power);
		}
		else if (cmd->type == CMDtype::set)
		{
			setPower(cmd->val);
		}
	}
	else if (cmd->cmd == "degrees")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += std::to_string(degreesOfRotation);
		}
		else if (cmd->type == CMDtype::set)
		{
			setDegrees(cmd->val);
		}
	}	
	else if (cmd->cmd == "esgain")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += std::to_string(endstop_gain);
		}
		else if (cmd->type == CMDtype::set)
		{
			setEndstopGain(cmd->val);
		}
	}
	else if (cmd->cmd == "invert")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += invertAxis ? "1" : "0";
		}
		else if (cmd->type == CMDtype::set)
		{
			invertAxis = cmd->val >= 1 ? true : false;
		}
	}
	else if (cmd->cmd == "idlespring")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += std::to_string(idlespringstrength);
		}
		else if (cmd->type == CMDtype::set)
		{
			setIdleSpringStrength(cmd->val);
		}
	}
	else if (cmd->cmd == "axisdamper")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += std::to_string(damperIntensity);
		}
		else if (cmd->type == CMDtype::set)
		{
			setDamperStrength(cmd->val);
		}
	}else{
		flag = ParseStatus::NOT_FOUND;
	}

	return flag;
};

