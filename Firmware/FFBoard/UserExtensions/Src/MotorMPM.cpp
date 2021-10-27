/*
 * MotorMPM.cpp
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#include <MotorMPM.h>

#define ADR_MPM_OFFSET	ADR_PWM_MODE

#define CPR	(1 << 16)

bool MotorMPM::mpmDriverInUse = false;


ClassIdentifier MotorMPM::info =
{ .name = "MPM", .id = 8, .hidden = false };


const ClassIdentifier MotorMPM::getInfo()
{
	return info;
}


MotorMPM::MotorMPM()
{
	MotorMPM::mpmDriverInUse = true;

	rx = true;
	encoderAngle = 0;
	lastEncoderAngle = 0;
	position = 0;
	rotation = 0;
	offset = 25989;
	aligned = false;
	spiErrors = 0;
	spiFrames = 0;

	torque = 0;
	spi = &HSPIDRV;

	restoreFlash();

	rx = false;
}


MotorMPM::~MotorMPM()
{
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
	torque = power;
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


int32_t MotorMPM::getPos()
{
	return position;
}


void MotorMPM::setPos(int32_t pos)
{
	aligned = false;
	rotation = 0;
	offset = pos - encoderAngle;
}


uint32_t MotorMPM::getCpr()
{
	return CPR;
}


void MotorMPM::exti(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == FLAG_Pin && !rx)
	{
		rx = true;

		spiFrames++;

		spiTx = enabled ? torque : 0; //(uint16_t)__builtin_bswap16(torque) : 0;

		if (HAL_SPI_TransmitReceive_IT(spi, (uint8_t*)(&spiTx), (uint8_t*)(&spiRx), 1) != HAL_OK)
		{
			spiErrors++;
			rx = false;
		}
	}
}


void MotorMPM::SpiTxRxCplt(SPI_HandleTypeDef *hspi)
{
	if (hspi == spi)
	{
		rawPosition = spiRx; //__builtin_bswap16(spiRx);

		encoderAngle = (int16_t)rawPosition;

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
			rotation = 0;
			aligned = true;
		}

		lastEncoderAngle = encoderAngle;

		position = (rotation * CPR) + encoderAngle + offset;

		rx = false;
	}
}


void MotorMPM::SpiError(SPI_HandleTypeDef *hspi)
{
	if (hspi == spi)
	{
		//HAL_SPI_Abort_IT(spi);
		spiErrors++;
		rx = false;
	}
}


ParseStatus MotorMPM::command(ParsedCommand *cmd, std::string *reply)
{
	ParseStatus result = ParseStatus::OK;

	if (cmd->cmd == "mpm")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += enabled ? "E" : "*";
			*reply +=
					" (" + std::to_string(rotation) + " * " + std::to_string(CPR)
					+ ") + " + std::to_string(encoderAngle)
					+ " + " + std::to_string(offset)
					+ " = " + std::to_string(position)
					+ "; Torque = " + std::to_string(torque)
					+ "; Raw = " + std::to_string(rawPosition)
					+ "; Frames = " + std::to_string(spiFrames)
					+ "; Errors = " + std::to_string(spiErrors);

			spiErrors = 0;
			spiFrames = 0;
		}
	}
	else
	{
		result = ParseStatus::NOT_FOUND; // No valid command
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
	else
	{
		spiErrors = 666;
	}
	aligned = false;
}
