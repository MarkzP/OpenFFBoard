/*
 * MotorMPM.h
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#ifndef MOTORMPM_H_
#define MOTORMPM_H_

#include "constants.h"
#include <vector>
#include "cppmain.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "ChoosableClass.h"
#include "PersistentStorage.h"
#include "CommandHandler.h"
#include "SpiHandler.h"
#include "thread.hpp"
#include "ExtiHandler.h"
#include "SPI.h"

#include "semaphore.hpp"
#include "OutputPin.h"
#include "cpp_target_config.h"


#define SPITIMEOUT 500
#define TMC_THREAD_MEM 512
#define TMC_THREAD_PRIO 25 // Must be higher than main thread


class MotorMPM: public MotorDriver, public PersistentStorage, public Encoder, public CommandHandler, public SPIDevice, public ExtiHandler {
public:
	MotorMPM();
	virtual ~MotorMPM();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();

	bool isCreatable();

	void turn(int16_t power);
	void stop();
	void start();

	int32_t getPos();
	void setPos(int32_t pos);

	uint32_t getCpr(); // Encoder counts per rotation

	bool hasIntegratedEncoder() { return true; }

	void exti(uint16_t GPIO_Pin);
	void spiTxRxCompleted(SPIPort* port);

	ParseStatus command(ParsedCommand* cmd,std::string* reply);

	void saveFlash();
	void restoreFlash();

	static bool mpmDriverInUse;

private:
	int32_t encoderAngle;
	int32_t lastEncoderAngle;
	int16_t torque;
	int32_t position;
	int32_t rotation;
	int32_t offset;
	bool aligned;

	volatile uint8_t spiTx[2];
	volatile uint8_t spiRx[2];

	volatile bool enabled = false;
	volatile bool initialized = false;
};

#endif /* MOTORMPM_H_ */
