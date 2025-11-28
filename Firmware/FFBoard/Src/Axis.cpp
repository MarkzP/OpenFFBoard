/*
 * Axis.cpp
 *
 *  Created on: 31.01.2020
 *      Author: Yannick
 */

#include "Axis.h"
#include "voltagesense.h"


//////////////////////////////////////////////
/*
 * Sources for class choosers are defined in MotorDriver and Encoder
 */


//////////////////////////////////////////////

ClassIdentifier Axis::info = {
	.name = "Axis",
	.id = CLSID_AXIS, // 1
	.visibility = ClassVisibility::visible};

Axis::Axis(char axis,volatile Control_t* control) :CommandHandler("axis", CLSID_AXIS), drv_chooser(MotorDriver::all_drivers),enc_chooser{Encoder::all_encoders}
{
	// Create HID FFB handler. Will receive all usb messages directly
	this->axis = axis;
	this->control = control;
	if (axis == 'X')
	{
		setInstance(0);
		this->flashAddrs = AxisFlashAddrs({ADR_AXIS1_CONFIG, ADR_AXIS1_MAX_SPEED, ADR_AXIS1_MAX_ACCEL,ADR_AXIS1_ENDSTOP, ADR_AXIS1_POWER, ADR_AXIS1_DEGREES,ADR_AXIS1_EFFECTS1});
	}
	else if (axis == 'Y')
	{
		setInstance(1);
		this->flashAddrs = AxisFlashAddrs({ADR_AXIS2_CONFIG, ADR_AXIS2_MAX_SPEED, ADR_AXIS2_MAX_ACCEL,ADR_AXIS2_ENDSTOP, ADR_AXIS2_POWER, ADR_AXIS2_DEGREES,ADR_AXIS2_EFFECTS1});
	}
	else if (axis == 'Z')
	{
		setInstance(2);
		this->flashAddrs = AxisFlashAddrs({ADR_AXIS3_CONFIG, ADR_AXIS3_MAX_SPEED, ADR_AXIS3_MAX_ACCEL,ADR_AXIS3_ENDSTOP, ADR_AXIS3_POWER, ADR_AXIS3_DEGREES,ADR_AXIS3_EFFECTS1});
	}


	restoreFlash(); // Load parameters
	CommandHandler::registerCommands(); // Internal commands
	registerCommands();

}

Axis::~Axis()
{

}

const ClassIdentifier Axis::getInfo() {

	return info;
}

void Axis::registerCommands(){
	registerCommand("power", Axis_commands::power, "Overall force strength",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("degrees", Axis_commands::degrees, "Rotation range in deg",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("esgain", Axis_commands::esgain, "Endstop stiffness",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("zeroenc", Axis_commands::zeroenc, "Zero axis",CMDFLAG_GET);
	registerCommand("invert", Axis_commands::invert, "Invert axis",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("idlespring", Axis_commands::idlespring, "Idle spring strength",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("axisdamper", Axis_commands::axisdamper, "Independent damper effect",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("enctype", Axis_commands::enctype, "Encoder type get/set/list",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("drvtype", Axis_commands::drvtype, "Motor driver type get/set/list",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("pos", Axis_commands::pos, "Encoder position",CMDFLAG_GET);
	registerCommand("notchf", Axis_commands::notchf, "Notch filter frequency *100",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("notchq", Axis_commands::notchq, "Notch filter Q *100",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("fxratio", Axis_commands::fxratio, "Effect ratio. Reduces effects excluding endstop. 255=100%",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("curtorque", Axis_commands::curtorque, "Axis torque",CMDFLAG_GET);
	registerCommand("curpos", Axis_commands::curpos, "Axis position",CMDFLAG_GET);
	registerCommand("delta", Axis_commands::delta_us, "Axis Time",CMDFLAG_GET);
}

/*
 * Read parameters from flash and restore settings
 */
void Axis::restoreFlash(){
	//NormalizedAxis::restoreFlash();
	// read all constants
	uint16_t value;
	if (Flash_Read(flashAddrs.config, &value)){
		this->conf = Axis::decodeConfFromInt(value);
	}else{
		pulseErrLed();
	}

	setDrvType(this->conf.drvtype);
	setEncType(this->conf.enctype);

	if (!Flash_Read(flashAddrs.notchf, &notchf)) notchf = 0;
	if (!Flash_Read(flashAddrs.notchq, &notchq)) notchq = 0;
	setNotchFilter();

	uint16_t esval, power;
	if(Flash_Read(flashAddrs.endstop, &esval)) {
		fx_ratio_i = esval & 0xff;
		endstopStrength = (esval >> 8) & 0xff;
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
// Saves parameters to flash.
void Axis::saveFlash(){
	//NormalizedAxis::saveFlash();
	Flash_Write(flashAddrs.config, Axis::encodeConfToInt(this->conf));
	Flash_Write(flashAddrs.notchf, notchf);
	Flash_Write(flashAddrs.notchq, notchq);
	Flash_Write(flashAddrs.endstop, fx_ratio_i | (endstopStrength << 8));
	Flash_Write(flashAddrs.power, power);
	Flash_Write(flashAddrs.degrees, (degreesOfRotation & 0x7fff) | (invertAxis << 15));
	Flash_Write(flashAddrs.effects1, idlespringstrength | (damperIntensity << 8));
}


uint8_t Axis::getDrvType(){
	return (uint8_t)this->conf.drvtype;
}

uint8_t Axis::getEncType(){
	if(drv->hasIntegratedEncoder()){
		return 255;
	}
	return (uint8_t)this->conf.enctype;
}


void Axis::setPos(uint16_t val)
{
	if(this->drv != nullptr){
		drv->getEncoder()->setPos(val);
	}
}

/*
 * Called from FFBWheel->Update() via AxesManager->Update()
 */
void Axis::prepareForUpdate(){
	if (drv == nullptr){
		pulseErrLed();
		return;
	}

	//if (!drv->motorReady()) return;

	double angle = getEncAngle(this->drv->getEncoder());

	// Scale encoder value to set rotation range
	// Update a change of range only when new range is within valid range
	// if degree change, compute the SpeedScaler, it depends on degreesOfRotation
	if (nextDegreesOfRotation != degreesOfRotation && abs(scaleEncValue(angle, nextDegreesOfRotation)) < 0x7fff){
		degreesOfRotation = nextDegreesOfRotation;

//		speedScalerNormalized = getNormalizedSpeedScaler(maxSpeedDegS, degreesOfRotation);
//		accelScalerNormalized = getNormalizedAccelScaler(maxAccelDegSS, degreesOfRotation);
	}


	// scaledEnc now gets inverted if necessary in updateMetrics
	int32_t scaledEnc = scaleEncValue(angle, degreesOfRotation);

	if (abs(scaledEnc) > 0xffff && drv->motorReady()){
		// We are way off. Shut down
		drv->stopMotor();
		pulseErrLed();
		if(!outOfBounds){
			outOfBounds = true;
			ErrorHandler::addError(outOfBoundsError);
		}

	}else if(abs(scaledEnc) <= 0x7fff) {
		outOfBounds = false;
		//ErrorHandler::clearError(outOfBoundsError);
	}

	this->updateMetrics(angle);

}

void Axis::errorCallback(const Error &error, bool cleared){
	if(cleared && error == this->outOfBoundsError){
		drv->startMotor();
		outOfBounds = false;
	}
}


void Axis::updateDriveTorque(){
	int32_t totalTorque = 0;
	updateTorque(&totalTorque);
	// Send to motor driver
	drv->turn((int16_t)totalTorque);
}

void Axis::setPower(uint16_t power)
{
	this->power = power;
	updateTorqueScaler();
#ifdef TMC4671DRIVER
	// Update hardware limits for TMC for safety
	TMC4671 *drv = dynamic_cast<TMC4671 *>(this->drv.get());
	if (drv != nullptr)
	{
		//tmclimits.pid_uq_ud = power;
		//tmclimits.pid_torque_flux = power;
		drv->setTorqueLimit(power);
	}
#endif
}


// create and setup a motor driver
void Axis::setDrvType(uint8_t drvtype)
{
	if (!drv_chooser.isValidClassId(drvtype))
	{
		return;
	}
	this->drv.reset(nullptr);
	MotorDriver* drv = drv_chooser.Create((uint16_t)drvtype);
	if (drv == nullptr)
	{
		return;
	}
	this->drv = std::unique_ptr<MotorDriver>(drv);
	this->conf.drvtype = drvtype;

	// Pass encoder to driver again
	if(!this->drv->hasIntegratedEncoder()){
		this->drv->setEncoder(this->enc);
	}

#ifdef TMC4671DRIVER
	if (dynamic_cast<TMC4671 *>(drv))
	{
		setupTMC4671();
	}
#endif

	if (!tud_connected())
	{
		control->usb_disabled = false;
		this->usbSuspend();
	}
	else
	{
		drv->startMotor();
	}
}

#ifdef TMC4671DRIVER
// Special tmc setup methods
void Axis::setupTMC4671()
{
	TMC4671 *drv = static_cast<TMC4671 *>(this->drv.get());
//	drv->setAxis(axis);
	drv->restoreFlash();
	tmclimits.pid_torque_flux = getPower();
	drv->setLimits(tmclimits);
	//drv->setBiquadFlux(fluxbq);
	drv->setExternalEncoderAllowed(true);
	

	// Enable driver

	drv->setMotionMode(MotionMode::torque);
	drv->Start(); // Start thread
}
#endif



/**
 * Init the encoder and reset metrics
 */
void Axis::setEncType(uint8_t enctype)
{
	if (enc_chooser.isValidClassId(enctype) && !drv->hasIntegratedEncoder())
	{

		this->conf.enctype = (enctype);
		this->enc = std::shared_ptr<Encoder>(enc_chooser.Create(enctype)); // Make new encoder
		if(drv && !drv->hasIntegratedEncoder())
			this->drv->setEncoder(this->enc);
	}else{
		this->conf.enctype = 0; // None encoder
	}

	double angle = getEncAngle(this->drv->getEncoder());
	//int32_t scaledEnc = scaleEncValue(angle, degreesOfRotation);
	// reset metrics
	this->resetMetrics(angle);

//	// init the speed/accel factor from default value
//	speedScalerNormalized = getNormalizedSpeedScaler(maxSpeedDegS, degreesOfRotation);
//	accelScalerNormalized = getNormalizedAccelScaler(maxAccelDegSS, degreesOfRotation);
}

/**
 * Returns a scaled encoder value between -0x7fff and 0x7fff with a range of degrees
 * Takes an encoder angle in degrees
 */

int32_t Axis::scaleEncValue(double angle, uint16_t degrees){
	if (degrees == 0){
		return 0x7fff;
	}

	int32_t val = (int32_t)(((double)0xffff / (double)degrees) * angle);

	return val;
}

/**
 * Returns the encoder position in degrees
 */
double Axis::getEncAngle(Encoder *enc){
	if(enc != nullptr){
		double pos = 360.0 * enc->getPos_f();
		if (isInverted()){
			pos= -pos;
		}
		return pos;
	}
	else{
		return 0;
	}
}


void Axis::emergencyStop(bool reset){
	drv->turn(0); // Send 0 torque first
	drv->emergencyStop(reset);
	//drv->stopMotor();
	control->emergency = !reset;
}

void Axis::usbSuspend(){
	if (drv != nullptr){
		drv->turn(0);
		drv->stopMotor();
	}
}

void Axis::usbResume(){
	if (drv != nullptr){
		drv->startMotor();
	}
}



metric_t* Axis::getMetrics() {
	return &metric.current;
}

//double Axis::getSpeedScalerNormalized() {
//	//return speedScalerNormalized;
//	return (double)0x7FFF / maxSpeedDegS;
//}

//double	 Axis::getAccelScalerNormalized() {
//	//return accelScalerNormalized;
//	return (double)0x7FFF / maxAccelDegSS;
//}


int32_t Axis::getLastScaledEnc() {
	return  clip(metric.current.pos,-0x7fff,0x7fff);
}


double Axis::updateIdleSpringForce() {
	return clip<double,double>((double)-metric.current.pos * idlespringscale, -idlespringclip, idlespringclip);
}

/*
 * Set the strength of the spring effect if FFB is disabled
 */
void Axis::setIdleSpringStrength(uint8_t spring){
	idlespringstrength = spring;
	if(spring == 0){
		idle_center = false;
	}else{
		idle_center = true;
	}
	idlespringclip = clip<double,double>((double)spring * 50.0, 0.0, 10000.0);
	idlespringscale = 0.5f + ((double)spring * 0.01f);
}

void Axis::setDamperStrength(uint8_t damper){
	this->damperIntensity = damper;
}

/*
 * Called before HID effects are calculated
 * Should calculate always on and idle effects specific to the axis like idlespring and friction
 */
void Axis::calculateAxisEffects(bool ffb_on){
	axisEffectTorque = 0.0;

	if(idle_center){
		axisEffectTorque += updateIdleSpringForce();
	}

	// Always active damper (more like friction)
	if(damperIntensity != 0){
		double dclip = (double)damperIntensity * 15.0;
		double damp = metric.current.speed * (double)damperIntensity * 0.5 / dclip;

		//damp = clip<double, double>(damp, -1.0, 1.0);

		constexpr double s1 = 0.0;
		constexpr double s2 = 1.0 - s1;
		damp *= (abs(damp) + s1) / ((damp * damp) + s2 * abs(damp) + 1.0);

		axisEffectTorque -= (damp * dclip);
	}

	// TODO: Always active inertia

}

void Axis::setNotchFilter() {
	if (notchf < 100 || notchf >= 50000 || notchq < 1 || notchq >= 1000) {
		notchFilter.setBiquad(BiquadType::bypass, 0.5, 1.0, 0.0);
	} else {
		notchFilter.setBiquad(BiquadType::notch, (double)notchf * 0.01, (double)notchq * 0.01, 0.0);
	}
}

void Axis::setFxRatio(uint8_t val) {
	fx_ratio_i = val;
}


void Axis::resetMetrics(double new_pos= 0) { // pos is degrees
	metric.current = metric_t();
	metric.current.posDegrees = new_pos;
	metric.current.pos = scaleEncValue(new_pos, degreesOfRotation);
	metric.previous = metric_t();
	// Reset filters
	speedFilter.reset();
	accelFilter.reset();
}


void Axis::updateMetrics(double new_pos) { // pos is degrees
	// store old value for next metric's computing
	metric.previous = metric.current;

	metric.current.time = DWT->CYCCNT;
	metric.current.delta = clip<double,double>((double)(metric.current.time - metric.previous.time) / (double)SystemCoreClock, 0.0005, 0.0015);

	metric.current.posDegrees = new_pos;
	int32_t scaled_pos = scaleEncValue(new_pos, degreesOfRotation);
	metric.current.pos = scaled_pos;

	metric.current.speedInstant = (new_pos - metric.previous.posDegrees) * 1000.0; // deg/s

	metric.current.speed = speedFilter.process(metric.current.speedInstant);

	metric.current.accelInstant = metric.current.speedInstant - metric.previous.speedInstant;
	metric.current.accel = accelFilter.process(metric.current.accelInstant); //accel_avg.getAverage(); //accel_avg.getAverage();

	metric.current.torque = 0;

//	if (calibrationInProgress) {
//		calibMaxSpeedNormalized = abs(metric.current.speed) > calibMaxSpeedNormalized ? abs(metric.current.speed) : calibMaxSpeedNormalized;
//		calibMaxAccelNormalized = abs(metric.current.accel) > calibMaxAccelNormalized ? abs(metric.current.accel) : calibMaxAccelNormalized;
//	}
}



uint16_t Axis::getPower(){
	return power;
}

void  Axis::updateTorqueScaler() {
	torqueScaler = ((double)power / (double)0x7fff);
}

double Axis::getTorqueScaler(){
	return torqueScaler;
}


int32_t Axis::getTorque() { return metric.current.torque; }

bool Axis::isInverted() {
	return invertAxis; // TODO store in flash
}

/**
 * Calculate soft endstop effect
 */
double Axis::updateEndstop(){

	double torque = 0.0;

	double dor = (double)this->degreesOfRotation * 0.5;
	double pos = metric.current.posDegrees;

	double overshoot = abs(pos) - dor;
	const double transition = 10.0;
	if (overshoot > 0.0)
	{
		double slope = 1.0;
		if (overshoot < transition)
		{
			double phaseRad = M_PI * overshoot / (2.0 * transition); // we start to compute the normalized angle (speed / normalizedSpeed@5%) and translate it of -1/2PI to translate sin on 1/2 periode
			slope = (1.0 + sin(phaseRad)) * 0.5;				// sin value is -1..1 range, we translate it to 0..2 and we scale it by 2
		}

		double springGain = (pos > 0.0 ? 1.0 : -1.0) * (double)endstopStrength * endstopGain * slope * torqueScaler;
		torque -= overshoot * springGain;

		double damperGain = (double)((fx_ratio_i - 102) / 4) * slope * torqueScaler;
		torque -= metric.current.speed * damperGain;
	}



	return torque;
}

void Axis::setEffectTorque(double torque) {

	if (abs(torque) > 0.0) {
		lastSetEffectTorque = HAL_GetTick();
		effectTorqueScaler = effectTorqueScaler >= 1.0 ? 1.0 : effectTorqueScaler + effectTorqueRamp;
	}
	effectTorque = torque;
}

// pass in ptr to receive the sum of the effects + endstop torque
// return true if torque is clipping
bool Axis::updateTorque(int32_t* totalTorque) {

	if ((HAL_GetTick() - lastSetEffectTorque) > effectTorqueTimeout) effectTorqueScaler = 0.0;

	double torque = axisEffectTorque * torqueScaler;
	torque += effectTorque * effectTorqueScaler * torqueScaler;
	torque += updateEndstop();

	torque = notchFilter.process(torque);
/*
	// TODO speed and accel limiters
	if(maxSpeedDegS > 0){

		double torqueSign = torque > 0 ? 1 : -1; // Used to prevent metrics against the force to go into the limiter
		// Speed. Mostly tuned...
		spdlimiterAvg.addValue(metric.current.speedInstant);
		double speedreducer = (double)((spdlimiterAvg.getAverage()*torqueSign) - (double)maxSpeedDegS) * getSpeedScalerNormalized();
		spdlimitreducerI = clip<double,int32_t>( spdlimitreducerI + ((speedreducer * 0.015) * torqueScaler),0,power);

		// Accel limit. Not really useful. Maybe replace with torque slew rate limit?
//		double accreducer = (double)((metric.current.accel*torqueSign) - (double)maxAccelDegSS) * getAccelScalerNormalized();
//		acclimitreducerI = clip<double,int32_t>( acclimitreducerI + ((accreducer * 0.02) * torqueScaler),0,power);


		// Only reduce torque. Don't invert it to prevent oscillation
		double torqueReduction = spdlimitreducerI + speedreducer * 0.025;// accreducer * 0.025 + acclimitreducerI
		if(torque > 0){
			torqueReduction = clip<double,int32_t>(torqueReduction,0,torque);
		}else{
			torqueReduction = clip<double,int32_t>(-torqueReduction,torque,0);
		}

		torque -= torqueReduction;
	}
	// Torque slew rate limiter
	if(maxTorqueRateMS > 0){
		torque = clip<int32_t,int32_t>(torque, metric.previous.torque - maxTorqueRateMS,metric.previous.torque + maxTorqueRateMS);
	}
	*/
//	if(torque - metric.previous.torque)
	if(outOfBounds){
		torque = 0.0;
	}

	// Torque calculated. Now sending to driver
	int32_t finalTorque = (int32_t)(invertAxis ? -torque : torque);
	finalTorque = clip<int32_t, int32_t>(finalTorque, -power, power);
	metric.current.torque = finalTorque;

	bool torqueChanged = finalTorque != metric.previous.torque;

	*totalTorque = finalTorque;
	return (torqueChanged);
}

void Axis::setDegrees(uint16_t degrees){

	degrees &= 0x7fff;
	if(degrees == 0){
		nextDegreesOfRotation = lastdegreesOfRotation;
	}else{
		lastdegreesOfRotation = degreesOfRotation;
		nextDegreesOfRotation = degrees;
	}
}


CommandStatus Axis::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<Axis_commands>(cmd.cmdId)){

	case Axis_commands::power:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(this->power);
		}
		else if (cmd.type == CMDtype::set)
		{
			setPower(cmd.val);
		}
		break;

	case Axis_commands::degrees:
		handleGetSetFunc(cmd, replies, degreesOfRotation, &Axis::setDegrees,this);
//		if (cmd.type == CMDtype::get)
//		{
//			replies.emplace_back(degreesOfRotation);
//		}
//		else if (cmd.type == CMDtype::set)
//		{
//			setDegrees(cmd.val);
//		}
		break;

	case Axis_commands::esgain:
		handleGetSet(cmd, replies, this->endstopStrength);
		break;

	case Axis_commands::zeroenc:
		this->setPos(0);
		break;

	case Axis_commands::invert:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(invertAxis ? 1 : 0);
		}
		else if (cmd.type == CMDtype::set)
		{
			invertAxis = cmd.val >= 1 ? true : false;
			resetMetrics(-metric.current.posDegrees);
		}
		break;

	case Axis_commands::idlespring:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(idlespringstrength);
		}
		else if (cmd.type == CMDtype::set)
		{
			setIdleSpringStrength(cmd.val);
		}
		break;

	case Axis_commands::axisdamper:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(damperIntensity);
		}
		else if (cmd.type == CMDtype::set)
		{
			setDamperStrength(cmd.val);
		}
		break;

	case Axis_commands::enctype:
		if(cmd.type == CMDtype::info){
			enc_chooser.replyAvailableClasses(replies,this->getEncType());
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(this->getEncType());
		}else if(cmd.type == CMDtype::set){
			this->setEncType(cmd.val);
		}
		break;

	case Axis_commands::drvtype:
		if(cmd.type == CMDtype::info){
			drv_chooser.replyAvailableClasses(replies,this->getDrvType());
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(this->getDrvType());
		}else if(cmd.type == CMDtype::set){
			this->setDrvType(cmd.val);
		}
		break;

	case Axis_commands::pos:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(this->drv->getEncoder()->getPos());
		}
		else if (cmd.type == CMDtype::set && this->drv->getEncoder() != nullptr)
		{
			this->drv->getEncoder()->setPos(cmd.val);
		}
		else
		{
			return CommandStatus::ERR;
		}
		break;

	case Axis_commands::notchf:
		handleGetSet(cmd, replies, this->notchf);
		if (cmd.type == CMDtype::set) setNotchFilter();
		break;

	case Axis_commands::notchq:
		handleGetSet(cmd, replies, this->notchq);
		if (cmd.type == CMDtype::set) setNotchFilter();
		break;

	case Axis_commands::fxratio:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->fx_ratio_i);
		}else if(cmd.type == CMDtype::set){
			setFxRatio(cmd.val);
		}
		break;

	case Axis_commands::curpos:
		replies.emplace_back(this->metric.current.pos);
		break;
	case Axis_commands::curtorque:
		replies.emplace_back(this->metric.current.torque);
		break;
	case Axis_commands::delta_us:
		replies.emplace_back(this->metric.current.delta * 1000000.0);
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}
	return CommandStatus::OK;
}



/*
 * Helper functions for encoding and decoding flash variables
 */
AxisConfig Axis::decodeConfFromInt(uint16_t val)
{
	// 0-6 enc, 7-12 Mot
	AxisConfig conf;
	conf.enctype = ((val)&0x3f);
	conf.drvtype = ((val >> 6) & 0x3f);
	return conf;
}

uint16_t Axis::encodeConfToInt(AxisConfig conf)
{
	uint16_t val = (uint8_t)conf.enctype & 0x3f;
	val |= ((uint8_t)conf.drvtype & 0x3f) << 6;
	return val;
}
