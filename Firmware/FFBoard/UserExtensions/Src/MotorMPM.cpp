/*
 * MotorMPM.cpp
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#include <MotorMPM.h>


#define CPR	(1 << 16)
#define MPM_OFFSET_ADR	0xfea

extern TIM_HandleTypeDef htim4;


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
	spi = &HSPIDRV;
	csport = SPI1_SS1_GPIO_Port;
	cspin = SPI1_SS1_Pin;
	timer_update = &htim4;

	restoreFlash();

	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);

	HAL_SPI_DeInit(spi);

	spi->Init.DataSize = SPI_DATASIZE_16BIT;
	spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	spi->Init.CLKPhase = SPI_PHASE_1EDGE;
	spi->Init.FirstBit = SPI_FIRSTBIT_LSB;
	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;

	HAL_SPI_Init(spi);

	initialized = true;
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
	torque = 0;
	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port,DRV_ENABLE_Pin,GPIO_PIN_RESET);
}


void MotorMPM::start()
{
	torque = 0;
	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port,DRV_ENABLE_Pin,GPIO_PIN_SET);
}


int32_t MotorMPM::getPos()
{
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


void MotorMPM::timerElapsed(TIM_HandleTypeDef* htim)
{
	if(htim == this->timer_update)
	{
		update();
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


void MotorMPM::update()
{
	if (!initialized || (spi->Instance->SR & SPI_SR_BSY) == SPI_SR_BSY)
	{
		return;
	}

	__disable_irq();

	spi->Instance->CR1 &= ~SPI_CR1_SPE;
	volatile uint32_t tmpdr = spi->Instance->DR;
	(void)tmpdr;

	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);

	spi->Instance->DR = (uint16_t)torque;
	spi->Instance->CR1 |= SPI_CR1_SPE;

	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);

	__enable_irq();

	while ((spi->Instance->SR & SPI_SR_BSY) == SPI_SR_BSY);

	if ((spi->Instance->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
	{
		encoderAngle = (int16_t)(spi->Instance->DR);

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


