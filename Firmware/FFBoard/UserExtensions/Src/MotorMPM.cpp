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
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);

	encoderAngle = 0;
	lastEncoderAngle = 0;
	position = 0;
	rotation = 0;
	offset = -11114;
	aligned = false;
	torque = 0;
	spi = &HSPIDRV;
	csport = SPI1_SS1_GPIO_Port;
	cspin = SPI1_SS1_Pin;

	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;

	spi->Instance->CR1 = (spi->Init.Mode | spi->Init.Direction | spi->Init.DataSize |
			spi->Init.CLKPolarity | spi->Init.CLKPhase | (spi->Init.NSS & SPI_CR1_SSM) |
			spi->Init.BaudRatePrescaler | spi->Init.FirstBit  | spi->Init.CRCCalculation);
}


MotorMPM::~MotorMPM()
{
	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port,DRV_ENABLE_Pin,GPIO_PIN_RESET);
}


void MotorMPM::turn(int16_t power)
{
	torque = power;
}


void MotorMPM::stop()
{
	enabled = false;

	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port,DRV_ENABLE_Pin,GPIO_PIN_RESET);

	torque = 0;
}


void MotorMPM::start()
{
	if (!initialized)
	{
		//restoreFlash();
		initialized = true;
	}

	torque = 0;

	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port,DRV_ENABLE_Pin,GPIO_PIN_SET);

	enabled = true;
}


int32_t MotorMPM::getPos()
{
	return position;
}


void MotorMPM::setPos(int32_t pos)
{
	if (initialized)
	{
		aligned = false;
		rotation = 0;
		offset = pos - encoderAngle;

		//saveFlash();
	}
}


uint32_t MotorMPM::getCpr()
{
	return CPR;
}


void MotorMPM::exti(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == FLAG_Pin)
	{
		if (enabled && initialized)
		{
			HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);

			uint16_t tmpTorque = (uint16_t)torque;

			spiTx[0] = (tmpTorque >> 8) & 0xff;
			spiTx[1] = tmpTorque & 0xff;

			if (HAL_SPI_TransmitReceive_DMA(spi, (uint8_t*)&spiTx, (uint8_t*)&spiRx, 2) != HAL_OK)
			{
				// Error condition
				HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);
			}
		}
	}
}


void MotorMPM::SpiTxRxCplt(SPI_HandleTypeDef *hspi)
{
	if (hspi == spi)
	{
		HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);

		int16_t tmpAngle = (int16_t)(((uint16_t)spiRx[0] << 8) + (uint16_t)spiRx[1]);
		encoderAngle = tmpAngle;

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
			aligned = true;
		}

		lastEncoderAngle = encoderAngle;

		position = (rotation * CPR) + encoderAngle + offset;
	}
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

