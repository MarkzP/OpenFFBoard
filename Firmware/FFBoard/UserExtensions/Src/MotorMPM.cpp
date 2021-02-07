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

	//spi->Init.DataSize = SPI_DATASIZE_16BIT;
	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;

	spi->Instance->CR1 = (spi->Init.Mode | spi->Init.Direction | spi->Init.DataSize |
			spi->Init.CLKPolarity | spi->Init.CLKPhase | (spi->Init.NSS & SPI_CR1_SSM) |
			spi->Init.BaudRatePrescaler | spi->Init.FirstBit  | spi->Init.CRCCalculation);

	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);

	state = MPM_MASTER_IDLE;
}


MotorMPM::~MotorMPM()
{
	HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port,DRV_ENABLE_Pin,GPIO_PIN_RESET);
}


void MotorMPM::turn(int16_t power)
{
	torque = power;

	update();
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
	update();

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
//	if(htim == this->timer_update)
//	{
//		if (state == MPM_MASTER_IDLE)
//		{
//			state = MPM_MASTER_UPDATE;
//		}
//	}
}


void MotorMPM::SpiTxRxCplt(SPI_HandleTypeDef *hspi)
{
	if (hspi == spi)
	{
		__disable_irq();

		if (state == MPM_MASTER_RXTX)
		{
			encoderAngle = (int16_t)(((uint16_t)spiRx[0] << 8) + (uint16_t)spiRx[1]);

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

			state = MPM_MASTER_IDLE;

			HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);
		}
		else
		{
			// Error condition
		}

		__enable_irq();
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
	__disable_irq();

	if (state == MPM_MASTER_IDLE)
	{

		state = MPM_MASTER_RXTX;
		HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);

		spiTx[0] = (torque >> 8) & 0xff;
		spiTx[1] = torque & 0xff;
#if 1
		if (HAL_SPI_TransmitReceive_DMA(spi, (uint8_t*)&spiTx, (uint8_t*)&spiRx, 2) != HAL_OK)
		{
			state = MPM_MASTER_IDLE;
			HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);
		}
#else
		if (HAL_SPI_TransmitReceive(spi, (uint8_t*)&spiTx, (uint8_t*)&spiRx, 2, 1) != HAL_OK)
		{
			state = MPM_MASTER_IDLE;
			//HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);
		}
		else
		{
			SpiTxRxCplt(spi);
		}
#endif
	}

	__enable_irq();
}
