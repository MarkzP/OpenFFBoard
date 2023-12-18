/*
 * MotorMPM.cpp
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#include <MotorMPM.h>

#define ADR_MPM_OFFSET	ADR_PWM_MODE
#define CLSID_MOT_MPM		0xF1

#define CPR	(1 << 16)

#if defined(STM32H743xx) || defined(STM32H723xx)
extern SPI_HandleTypeDef hspi3;
#else
extern SPI_HandleTypeDef hspi1;
#endif

bool MotorMPM::mpmDriverInUse = false;


ClassIdentifier MotorMPM::info =
{ .name = "MPM", .id = CLSID_MOT_MPM, };


const ClassIdentifier MotorMPM::getInfo()
{
	return info;
}


MotorMPM::MotorMPM() : CommandHandler("mpmdrv", CLSID_MOT_MPM)
{
	MotorMPM::mpmDriverInUse = true;

	encoderAngle = 0;
	lastEncoderAngle = 0;
	position = 0;
	rotation = 0;
	offset = 25989;
	aligned = false;
	torque = 0;

	spi = &hspi3;

	restoreFlash();

	ready = true;

	CommandHandler::registerCommands();
	registerCommand("info", MotorMPM_commands::info, "MPM info", CMDFLAG_GET | CMDFLAG_INFOSTRING);
}


MotorMPM::~MotorMPM()
{
	ready = false;
	MotorMPM::mpmDriverInUse = false;
}

bool MotorMPM::isCreatable()
{
	return !MotorMPM::mpmDriverInUse;
}

bool MotorMPM::hasIntegratedEncoder()
{
	return true;
}

Encoder* MotorMPM::getEncoder()
{
	return static_cast<Encoder*>(this);
}

void MotorMPM::turn(int16_t power)
{
	torque = enabled ? power : 0;

	if (ready) HAL_GPIO_WritePin(OUT_MPM_SS_GPIO_Port, OUT_MPM_SS_Pin, GPIO_PIN_SET);
}


void MotorMPM::stopMotor()
{
	enabled = false;

	torque = 0;
}


void MotorMPM::startMotor()
{
	torque = 0;

	enabled = true;
}


bool MotorMPM::motorReady()
{
	return ready;
}


int32_t MotorMPM::getPos()
{
	if (positionChanged)
	{
		encoderAngle = (int16_t)rawPosition;

		positionChanged = false;

		if (ready) HAL_GPIO_WritePin(OUT_MPM_SS_GPIO_Port, OUT_MPM_SS_Pin, GPIO_PIN_SET);

		if (aligned)
		{
			int32_t delta =  encoderAngle - lastEncoderAngle;

			if (delta > (CPR / 2))
			{
				rotation--;
			}
			else if (delta < -(CPR / 2))
			{
				rotation++;
			}
		}
		else
		{
			rotation = rawPosition < offset ? -1 : 0;
			aligned = true;
		}

		lastEncoderAngle = encoderAngle;

		position = (rotation * CPR) + encoderAngle + offset;
	}

	return position;
}


void MotorMPM::setPos(int32_t pos)
{
	aligned = false;
	rotation = 0;
	offset = pos - encoderAngle;

	if (ready) HAL_GPIO_WritePin(OUT_MPM_SS_GPIO_Port, OUT_MPM_SS_Pin, GPIO_PIN_SET);
}


uint32_t MotorMPM::getCpr()
{
	return CPR;
}


void MotorMPM::exti(uint16_t GPIO_Pin)
{
	if (GPIO_Pin != IN_MPM_INT_Pin) return;

	HAL_GPIO_WritePin(OUT_MPM_SS_GPIO_Port, OUT_MPM_SS_Pin, GPIO_PIN_RESET);

	if (!ready) return;

	spiTx = torque;

	HAL_SPI_TransmitReceive_IT(spi, (uint8_t*)(&spiTx), (uint8_t*)(&spiRx), 1);
}


void MotorMPM::SpiTxRxCplt(SPI_HandleTypeDef *hspi)
{
	if (hspi != spi) return;

	rawPosition = spiRx;

	positionChanged = true;
}


void MotorMPM::SpiError(SPI_HandleTypeDef *hspi)
{
	if (hspi != spi) return;

	HAL_SPI_Abort_IT(spi);

	positionChanged = true;
}


CommandStatus MotorMPM::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies)
{
	CommandStatus result = CommandStatus::OK;

	if (cmd.cmdId == (uint32_t)MotorMPM_commands::info)
	{
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(
					"OK ; Rdy=" + std::to_string(ready)
					+ "; Enabled=" + std::to_string(enabled)
					+ "; SS=" + std::to_string(HAL_GPIO_ReadPin(OUT_MPM_SS_GPIO_Port, OUT_MPM_SS_Pin))
					+ " (" + std::to_string(rotation) + " * " + std::to_string(CPR)
					+ ") + " + std::to_string(encoderAngle)
					+ " + " + std::to_string(offset)
					+ " = " + std::to_string(position)
					+ "; Torque=" + std::to_string(torque)
					+ "; Raw=" + std::to_string(rawPosition)
			);
		}
	}
	else
	{
		result = CommandStatus::NOT_FOUND; // No valid command
	}

	return result;
}


void MotorMPM::saveFlash()
{
	uint16_t u_offset = (uint16_t)offset;
	Flash_Write(ADR_MPM_OFFSET, u_offset);
}


void MotorMPM::restoreFlash()
{
	uint16_t u_offset;
	if (Flash_Read(ADR_MPM_OFFSET, &u_offset))
	{
		offset = (int16_t)u_offset;
	}

	aligned = false;
}
