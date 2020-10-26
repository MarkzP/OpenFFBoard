/*
 * MotorMPM.h
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#ifndef MOTORMPM_H_
#define MOTORMPM_H_

#include <MotorDriver.h>
#include <Encoder.h>
#include <CommandHandler.h>


class MotorMPM: public MotorDriver, public Encoder, public CommandHandler {
public:
	MotorMPM();
	virtual ~MotorMPM();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();

	void turn(int16_t power);
	void stop();
	void start();

	int32_t getPos();
	void setPos(int32_t pos);

	uint32_t getCpr(); // Encoder counts per rotation

	ParseStatus command(ParsedCommand* cmd,std::string* reply);

	void saveFlash();
	void restoreFlash();

private:
	int32_t encoderAngle;
	int32_t lastEncoderAngle;
	int16_t torque;
	int32_t position;
	int32_t rotation;
	int32_t offset;
	uint8_t aligned;

	void spiRxTx();
};

#endif /* MOTORMPM_H_ */
