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
{ .name = "MPM", .id = CLSID_MOT_PWM, };


const ClassIdentifier MotorMPM::getInfo()
{
	return info;
}


MotorMPM::MotorMPM() : CommandHandler("mpmdrv", CLSID_MOT_PWM)
{
	MotorMPM::mpmDriverInUse = true;

	encoderAngle = 0;
	lastEncoderAngle = 0;
	position = 0;
	rotation = 0;
	offset = 25989;
	aligned = false;
	spiTxErrors = 0;
	spiRxErrors = 0;
	spiNbr = 0;
	ssiNbr = 0;
	extiNbr = 0;

	torque = 0;
	spi = &HSPIDRV;

	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	HAL_SPI_Init(spi);

	restoreFlash();

	ready = true;

	CommandHandler::registerCommands();
	registerCommand("mpm", MotorMPM_commands::mpm, "MPM info", CMDFLAG_GET | CMDFLAG_INFOSTRING);
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

	HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_SET);

	ssiNbr++;
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

		HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_SET);

		ssiNbr++;

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
	}

	return position;
}


void MotorMPM::setPos(int32_t pos)
{
	aligned = false;
	rotation = 0;
	offset = pos - encoderAngle;

	HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_SET);

	ssiNbr++;
}


uint32_t MotorMPM::getCpr()
{
	return CPR;
}


void MotorMPM::exti(uint16_t GPIO_Pin)
{
	if (GPIO_Pin != FLAG_Pin) return;

	if (!ready) return;

	extiNbr++;

	spiTx = __builtin_bswap16(torque);

	if (HAL_SPI_TransmitReceive_DMA(spi, (uint8_t*)(&spiTx), (uint8_t*)(&spiRx), 2) != HAL_OK)
	{
		spiTxErrors++;

		HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_RESET);
	}
}


void MotorMPM::SpiTxRxCplt(SPI_HandleTypeDef *hspi)
{
	if (hspi != spi) return;

	spiNbr++;

	rawPosition = __builtin_bswap16(spiRx);

	positionChanged = true;

	HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_RESET);
}


void MotorMPM::SpiError(SPI_HandleTypeDef *hspi)
{
	if (hspi == spi)
	{
		HAL_SPI_Abort_IT(spi);
		spiRxErrors++;

		HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin, GPIO_PIN_RESET);
	}
}


CommandStatus MotorMPM::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies)
{
	CommandStatus result = CommandStatus::OK;

	if (cmd.cmdId == (uint32_t)MotorMPM_commands::mpm)
	{
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(
					"OK ; Rdy=" + std::to_string(ready)
					+ "; En=" + std::to_string(enabled)
					+ "; SS1=" + std::to_string(HAL_GPIO_ReadPin(SPI1_SS1_GPIO_Port, SPI1_SS1_Pin))
					+ " (" + std::to_string(rotation) + " * " + std::to_string(CPR)
					+ ") + " + std::to_string(encoderAngle)
					+ " + " + std::to_string(offset)
					+ " = " + std::to_string(position)
					+ "; Torque=" + std::to_string(torque)
					+ "; Raw=" + std::to_string(rawPosition)
					+ "; Exti=" + std::to_string(extiNbr)
					+ "; Ssi=" + std::to_string(ssiNbr)
					+ "; Spi=" + std::to_string(spiNbr)
					+ "; RxErr=" + std::to_string(spiRxErrors)
					+ "; TxErr=" + std::to_string(spiTxErrors)
			);

//			extiNbr = 0;
//			ssiNbr = 0;
//			spiNbr = 0;
//			spiTxErrors = 0;
//			spiRxErrors = 0;
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
	else
	{
		spiTxErrors = 666;
	}
	aligned = false;
}
