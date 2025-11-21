/*
 * MotorMPM.h
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#ifndef MOTORMPM_H_
#define MOTORMPM_H_

#include <MotorDriver.h>
#include <ExtiHandler.h>
#include <SpiHandler.h>
#include <PersistentStorage.h>
#include <Encoder.h>
#include <CommandHandler.h>


class MotorMPM: public MotorDriver,
		public ExtiHandler,
		public SpiHandler,
		public PersistentStorage,
		public Encoder,
		public CommandHandler {
	enum class MotorMPM_commands : uint32_t {
		info,
	};
public:
	MotorMPM();
	virtual ~MotorMPM();

	static ClassIdentifier info;
	const ClassIdentifier getInfo();
	static bool isCreatable();

	Encoder* getEncoder() override;
	bool hasIntegratedEncoder() override;

	void turn(int16_t power);
	void stopMotor();
	void startMotor();

	bool motorReady();

	int32_t getPos();
	void setPos(int32_t pos);

	uint32_t getCpr(); // Encoder counts per rotation

	void exti(uint16_t GPIO_Pin);
	void SpiTxRxCplt(SPI_HandleTypeDef *hspi);
	void SpiError(SPI_HandleTypeDef *hspi);

	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);
	virtual std::string getHelpstring(){return "MPM SPI motor driver";}

	void saveFlash();
	void restoreFlash();

	static bool mpmDriverInUse;

private:
	uint16_t rawPosition;
	int32_t encoderAngle;
	int32_t lastEncoderAngle;
	int16_t torque;
	float rawTorque;
	int32_t position;
	int32_t rotation;
	int32_t offset;
	bool ready = false;
	bool sync = false;
	bool aligned = false;
	bool positionChanged = false;

	int spiErrors = 0;

	SPI_HandleTypeDef *spi;

	volatile uint16_t spiTx;
	volatile uint16_t spiRx;

	volatile bool enabled = false;
};

#endif /* MOTORMPM_H_ */
