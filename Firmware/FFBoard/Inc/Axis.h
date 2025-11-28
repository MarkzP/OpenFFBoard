/*
 * Axis.h
 *
 *  Created on: 21.01.2021
 *      Author: Yannick / Lidders
 */

#ifndef SRC_AXIS_H_
#define SRC_AXIS_H_
#include <FFBoardMain.h>
#include <MotorPWM.h>
#include "usb_hid_ffb_desc.h"
#include "TMC4671.h"
#include "PersistentStorage.h"
#include "ButtonSource.h"
#include "EncoderLocal.h"
#include "LocalAnalog.h"
#include "AnalogSource.h"

#include "HidFFB.h"
#include "ffb_defs.h"
#include "TimerHandler.h"
#include "ClassChooser.h"
#include "ExtiHandler.h"
#include "EffectsCalculator.h"
#include "FastAvg.h"


struct Control_t {
	bool emergency = false;
	bool usb_disabled = true;
	bool update_disabled = true;
	bool request_update_disabled = false;
	bool resetEncoder = false;
};

struct AxisFlashAddrs
{
	uint16_t config = ADR_AXIS1_CONFIG;
	uint16_t notchf = ADR_AXIS1_MAX_SPEED;
	uint16_t notchq = ADR_AXIS1_MAX_ACCEL;
	uint16_t endstop = ADR_AXIS1_ENDSTOP;

	uint16_t power = ADR_AXIS1_POWER;
	uint16_t degrees = ADR_AXIS1_DEGREES;
	uint16_t effects1 = ADR_AXIS1_EFFECTS1;
};

struct AxisConfig
{
	uint8_t drvtype = 0;
	uint8_t enctype = 0;
	//bool invert = false;
};
struct metric_t {
	uint32_t time = 0;
	double delta = 0.0;
	double accel = 0;	// in deg/s²
	double accelInstant = 0;
	double speed = 0;
	double speedInstant = 0; // in deg/s
	int32_t pos = 0;
	double posDegrees = 0;
	int32_t torque = 0; // total of effect + endstop torque
};


struct axis_metric_t {
	metric_t current;
	metric_t previous;
};


enum class Axis_commands : uint32_t{
	power=0x00,degrees=0x01,esgain,zeroenc,invert,idlespring,axisdamper,enctype,drvtype,pos,notchf,notchq,fxratio,curtorque,curpos,delta_us
};

class Axis : public PersistentStorage, public CommandHandler, public ErrorHandler
{
public:
	Axis(char axis, volatile Control_t* control);
	virtual ~Axis();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();
	const ClassType getClassType() override {return ClassType::Axis;};

	virtual std::string getHelpstring() { return "FFB axis"	;}
#ifdef TMC4671DRIVER
	void setupTMC4671();
#endif

	// Dynamic classes
	void setDrvType(uint8_t drvtype);
	void setEncType(uint8_t enctype);
	uint8_t getDrvType();
	uint8_t getEncType();

	void usbSuspend(); // Called on usb disconnect and suspend
	void usbResume();  // Called on usb resume

	void saveFlash() override;
	void restoreFlash() override;

	void prepareForUpdate();  // called before the effects are calculated
	void updateDriveTorque(); //int32_t effectTorque);
	void emergencyStop(bool reset);

	void setPos(uint16_t val);
	void zeroPos();

	bool getFfbActive();

	int32_t scaleEncValue(double angle, uint16_t degrees);
	double 	getEncAngle(Encoder *enc);
//	double	getNormalizedSpeedScaler(uint16_t maxSpeedRpm, uint16_t degrees);
//	double	getNormalizedAccelScaler(uint16_t maxAccelRpm, uint16_t degrees);
//	double	getSpeedFromNormalized(uint16_t speedNormalized, uint16_t degrees);
//	double	getAccelFromNormalized(uint16_t accelNormalized, uint16_t degrees);


	void setPower(uint16_t power);


	void errorCallback(const Error &error, bool cleared) override;

	//ParseStatus command(ParsedCommand_old* cmd,std::string* reply) override;
	void registerCommands();
	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);

	ClassChooser<MotorDriver> drv_chooser;
	ClassChooser<Encoder> enc_chooser;

	int32_t getLastScaledEnc();
	void resetMetrics(double new_pos);
	void updateMetrics(double new_pos);
	double updateIdleSpringForce();
	void setIdleSpringStrength(uint8_t spring);
	void setDamperStrength(uint8_t damper);
	void calculateAxisEffects(bool ffb_on);
	int32_t getTorque(); // current torque scaled as a 32 bit signed value
	double updateEndstop();

	metric_t* getMetrics();
	//double 	 getSpeedScalerNormalized();
	//double	 getAccelScalerNormalized();

	const uint32_t effectTorqueTimeout = 5000;
	const double effectTorqueRamp = 1.0 / 500.0;
	double effectTorqueScaler = 0.0;
	uint32_t lastSetEffectTorque;
	void setEffectTorque(double torque);
	bool updateTorque(int32_t* totalTorque);



private:
	AxisFlashAddrs flashAddrs;
	volatile Control_t* control;

	//TIM_HandleTypeDef *timer_update;
	AxisConfig conf;

	std::unique_ptr<MotorDriver> drv = std::make_unique<MotorDriver>(); // dummy
	std::shared_ptr<Encoder> enc = nullptr;

	bool outOfBounds = false;

	static AxisConfig decodeConfFromInt(uint16_t val);
	static uint16_t encodeConfToInt(AxisConfig conf);

	const Error outOfBoundsError = Error(ErrorCode::axisOutOfRange,ErrorType::warning,"Axis out of bounds");

#ifdef TMC4671DRIVER
	const TMC4671PIDConf tmcpids = TMC4671PIDConf({.fluxI = 400,
											 .fluxP = 400,
											 .torqueI = 400,
											 .torqueP = 300,
											 .velocityI = 0,
											 .velocityP = 128,
											 .positionI = 0,
											 .positionP = 64,
											 .sequentialPI = true});
	TMC4671Limits tmclimits = TMC4671Limits({.pid_torque_flux_ddt = 32767,
											 .pid_uq_ud = 30000,
											 .pid_torque_flux = 30000,
											 .pid_acc_lim = 2147483647,
											 .pid_vel_lim = 2147483647,
											 .pid_pos_low = -2147483647,
											 .pid_pos_high = 2147483647});

	// Lowpass 1KHZ
	const TMC4671Biquad fluxbq = TMC4671Biquad({.a1 = -134913,
										  .a2 = 536735999,
										  .b0 = 67457,
										  .b1 = 134913,
										  .b2 = 67457,
										  .enable = true});
#endif


	double encoderOffset = 0; // Offset for absolute encoders
	uint16_t degreesOfRotation = 900;					// How many degrees of range for the full gamepad range
	uint16_t lastdegreesOfRotation = degreesOfRotation; // Used to store the previous value
	uint16_t nextDegreesOfRotation = degreesOfRotation; // Buffer when changing range

	// Limiters
	//uint16_t maxSpeedDegS  = 0; // Set to non zero to enable. example 1000. 8b * 10?
	//double	 maxAccelDegSS = 0;
	//uint32_t maxTorqueRateMS = 0; // 8b * 128?

	//double spdlimitreducerI = 0;
	//double acclimitreducerI = 0;
	//const uint8_t accelFactor = 10.0; // Conversion factor between internal and external acc limit

//	bool	 calibrationInProgress;
//	uint16_t calibMaxSpeedNormalized;
//	double	 calibMaxAccelNormalized;

	void setDegrees(uint16_t degrees);

	uint16_t getPower();
	double getTorqueScaler();
	bool isInverted();
	char axis;


	// Merge normalized
	axis_metric_t metric;
	double effectTorque = 0;
	double axisEffectTorque = 0;
	uint8_t fx_ratio_i = 204; // Reduce effects to a certain ratio of the total power to have a margin for the endstop. 80% = 204
	uint16_t power = 2000;
	double torqueScaler = 0; // power * fx_ratio as a ratio between 0 & 1
	bool invertAxis = false;
	uint8_t endstopStrength = 127; // Sets how much extra torque per count above endstop is added. High = stiff endstop. Low = softer
	const double endstopGain = 50.0; // Overall max endstop intensity

	uint8_t idlespringstrength = 127;
	double idlespringclip = 0.0f;
	double idlespringscale = 0.0f;
	bool idle_center = false;

	double speed_f = 20.0 , speed_q = 0.7;
	double accel_f = 120.0 , accel_q = 0.3;
	const double filter_f = 1000.0f; // 1khz
	const double damperClip = 15000.0f;
	uint8_t damperIntensity = 30;
	Biquad speedFilter = Biquad(BiquadType::lowpass, speed_f/filter_f, speed_q, 0.0);
	Biquad accelFilter = Biquad(BiquadType::lowpass, accel_f/filter_f, accel_q, 0.0);
	//Biquad limitsFilter = Biquad(BiquadType::lowpass, 20/filter_f, 0.4, 0.0);
	FastAvg<double,8> spdlimiterAvg;

	uint16_t notchf = 0;
	uint16_t notchq = 0;
	Biquad notchFilter = Biquad(BiquadType::bypass, 0.5f, 1.0f, 0.0f);
	void setNotchFilter();

	void setFxRatio(uint8_t val);
	void updateTorqueScaler();
};

#endif /* SRC_AXIS_H_ */
