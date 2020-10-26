/*
 * MotorMPM.cpp
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#include <MotorMPM.h>



#define CPR	(1 << 16)


#define MPM_OFFSET_ADR	0xfea


ClassIdentifier MotorMPM::info =
{ .name = "MPM", .id = 3, .hidden = false };

const ClassIdentifier MotorMPM::getInfo()
{
	return info;
}

MotorMPM::MotorMPM()
{
	encoderAngle = 0;
	lastEncoderAngle = 0;
	position = 0;
	rotation = 0;
	offset = 4530;
	aligned = false;
	torque = 0;

	restoreFlash();

	HAL_SPI_DeInit(&hspi2);

	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;

	HAL_SPI_Init(&hspi2);
}

MotorMPM::~MotorMPM()
{

}

void MotorMPM::turn(int16_t power)
{
	torque = power;

	//spiRxTx();
}

void MotorMPM::stop()
{
	torque = 0;

	spiRxTx();
}

void MotorMPM::start()
{
	torque = 0;
}

int32_t MotorMPM::getPos()
{
	spiRxTx();

	return position;
}

void MotorMPM::setPos(int32_t pos)
{
	__disable_irq();
	aligned = false;
	rotation = 0;
	offset = pos - encoderAngle;
	__enable_irq();

	saveFlash();
}

uint32_t MotorMPM::getCpr()
{
	return CPR;
}

ParseStatus MotorMPM::command(ParsedCommand *cmd, std::string *reply)
{
	ParseStatus result = ParseStatus::OK;

	if (cmd->cmd == "mpenc")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply +=
					"(" + std::to_string(rotation) + " * " + std::to_string(CPR)
					+ ") + " + std::to_string(encoderAngle)
					+ " + " + std::to_string(offset)
					+ " = " + std::to_string(position);
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
	Flash_Write(MPM_OFFSET_ADR, u_offset);
}

void MotorMPM::restoreFlash()
{
	uint16_t u_offset;
	Flash_Read(MPM_OFFSET_ADR, &u_offset);
	offset = (int16_t)u_offset;
}


void MotorMPM::spiRxTx()
{
	bool updated = false;

	__disable_irq();
	SPI2->CR1 |= SPI_CR1_SPE;
	SPI2->DR = torque;
	for (int i = 0; i < 16; ++i)
	{
		if ((SPI2->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
		{
			encoderAngle = (int16_t)(SPI2->DR);
			updated = true;
			break;
		}
	}
	__enable_irq();


	if (!updated)
	{
		return;
	}

	if (aligned)
	{
		int32_t delta =  encoderAngle - lastEncoderAngle;

		if (abs(delta) > (CPR / 2))
		{
			if (delta > 0)
			{
				rotation--;
			}
			else
			{
				rotation++;
			}
		}
	}

	aligned = true;
	lastEncoderAngle = encoderAngle;

	position = (rotation * CPR) + encoderAngle + offset;
}


