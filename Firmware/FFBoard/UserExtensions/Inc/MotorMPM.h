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

	// MotorDriver
	static ClassIdentifier info;
	const ClassIdentifier getInfo();
	static bool isCreatable();

	Encoder* getEncoder() override;
	bool hasIntegratedEncoder() override;

	void turn(int16_t power) override;
	void stopMotor() override;
	void startMotor() override;
	bool motorReady() override;

	// Encoder
	int32_t getPos() override;
	double getPos_f() override;
	void setPos(int32_t pos) override;
	uint32_t getCpr() override; // Encoder counts per rotation

	// ExtiHandler
	void exti(uint16_t GPIO_Pin) override;

	// SpiHandler
	void SpiTxRxCplt(SPI_HandleTypeDef *hspi) override;
	void SpiError(SPI_HandleTypeDef *hspi) override;

	// CommandHandler
	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies) override;
	virtual std::string getHelpstring() override {return "MPM SPI motor driver";}

	// PersistentStorage
	void saveFlash() override;
	void restoreFlash() override;

private:
	static bool mpmDriverInUse;

	int32_t encoderAngle;
	int32_t lastEncoderAngle;
	int16_t torque;
	int32_t rotation;
	int32_t offset;

	static constexpr int realignCount = 5;
	int align = realignCount;
	inline void dealign() { align = realignCount; }
	inline bool aligned() { return align == 0; }
	inline bool alignCompleted() { return --align == 0; }

	SPI_HandleTypeDef *spi;

	int32_t position;
	uint16_t spiTx;
	uint16_t spiRx;

	int spiErrors = 0;
	bool enabled = false;

	TaskHandle_t xTaskToNotify = nullptr;
};

#endif /* MOTORMPM_H_ */
