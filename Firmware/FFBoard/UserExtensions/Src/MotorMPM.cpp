/*
 * MotorMPM.cpp
 *
 *  Created on: Oct 2, 2020
 *      Author: Marc
 */

#include <MotorMPM.h>


#define CPR	(1 << 16)


ClassIdentifier MotorMPM::info =
{ .name = "MPM", .id = 20, .unique = '0', .hidden = false };


const ClassIdentifier MotorMPM::getInfo()
{
	return info;
}

bool MotorMPM::isCreatable(){
	return !MotorMPM::mpmDriverInUse; // Creatable if not already in use for example by another axis
}


MotorMPM::MotorMPM() : SPIDevice(motor_spi, OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin))
{
	MotorMPM::mpmDriverInUse = true;
	encoderAngle = 0;
	lastEncoderAngle = 0;
	position = 0;
	rotation = 0;
	offset = -11114;
	aligned = false;
	torque = 0;
	//spi = &HSPIDRV;
	//csport = SPI1_SS1_GPIO_Port;
	//cspin = SPI1_SS1_Pin;

//	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
//
//	spi->Instance->CR1 = (spi->Init.Mode | spi->Init.Direction | spi->Init.DataSize |
//			spi->Init.CLKPolarity | spi->Init.CLKPhase | (spi->Init.NSS & SPI_CR1_SSM) |
//			spi->Init.BaudRatePrescaler | spi->Init.FirstBit  | spi->Init.CRCCalculation);

	spiConfig.peripheral.Mode = SPI_MODE_MASTER;
	spiConfig.peripheral.Direction = SPI_DIRECTION_2LINES;
	spiConfig.peripheral.DataSize = SPI_DATASIZE_8BIT;
	spiConfig.peripheral.CLKPolarity = SPI_POLARITY_HIGH;
	spiConfig.peripheral.CLKPhase = SPI_PHASE_2EDGE;
	spiConfig.peripheral.NSS = SPI_NSS_SOFT;
	spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	spiConfig.peripheral.FirstBit = SPI_FIRSTBIT_MSB;
	spiConfig.peripheral.TIMode = SPI_TIMODE_DISABLE;
	spiConfig.peripheral.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spiConfig.peripheral.CRCPolynomial = 10;
	spiConfig.cspol = true;

	spiPort.takeSemaphore();
	spiPort.configurePort(&spiConfig.peripheral);
	spiPort.giveSemaphore();
}


MotorMPM::~MotorMPM()
{
	MotorMPM::mpmDriverInUse = false;
}


void MotorMPM::turn(int16_t power)
{
	torque = power;
}


void MotorMPM::stop()
{
	enabled = false;

	torque = 0;
}


void MotorMPM::start()
{
	if (!initialized)
	{
		restoreFlash();
		initialized = true;
	}

	torque = 0;

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

		saveFlash();
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
			uint16_t tmpTorque = (uint16_t)torque;

			spiTx[0] = (tmpTorque >> 8) & 0xff;
			spiTx[1] = tmpTorque & 0xff;

			spiPort.transmitReceive_DMA((uint8_t*)&spiTx, (uint8_t*)&spiRx, 2, this);
		}
	}
}


void MotorMPM::spiTxRxCompleted(SPIPort* port)
{
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


ParseStatus MotorMPM::command(ParsedCommand *cmd, std::string *reply)
{
	ParseStatus result = ParseStatus::OK;

	if (cmd->cmd == "mpenc")
	{
		if (cmd->type == CMDtype::get)
		{
			*reply += initialized ? "I" : "*";
			*reply += enabled ? "E" : "*";
			*reply +=
					" (" + std::to_string(rotation) + " * " + std::to_string(CPR)
					+ ") + " + std::to_string(encoderAngle)
					+ " + " + std::to_string(offset)
					+ " = " + std::to_string(position)
					+ "; Torque = " + std::to_string(torque);
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
	Flash_Write(ADR_PWM_MODE, u_offset);
}


void MotorMPM::restoreFlash()
{
	//uint16_t u_offset;
	//Flash_Read(ADR_PWM_MODE, &u_offset);
	offset = -27944; //(int16_t)u_offset;
}

