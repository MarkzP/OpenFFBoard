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
#include <TimerHandler.h>
#include <CommandHandler.h>


class MotorMPM: public MotorDriver, public Encoder, public TimerHandler, public CommandHandler {
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

	void timerElapsed(TIM_HandleTypeDef* htim);

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

	TIM_HandleTypeDef* timer_update;
	SPI_HandleTypeDef *spi;
	GPIO_TypeDef *csport;
	uint16_t cspin;

	void update();
};

#endif /* MOTORMPM_H_ */
