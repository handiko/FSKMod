/*
 * FSKMod.cpp
 *
 *  Created on: Dec 2, 2020
 *      Author: handiko
 */

#include "FSKMod.hpp"
#include "main.h"

FSKMod::FSKMod() {
	// TODO Auto-generated constructor stub
	reset();

	setChAFreq(161975000UL);
	setChBFreq(162025000UL);
	setIdleFreq(1000000UL);
}

/*
 * Private methods
 */
void FSKMod::initGPIOClock(void)
{
#if defined(GPIOA)
	__HAL_RCC_GPIOA_CLK_ENABLE();
#endif

#if defined(GPIOB)
	__HAL_RCC_GPIOB_CLK_ENABLE();
#endif

#if defined(GPIOC)
	__HAL_RCC_GPIOC_CLK_ENABLE();
#endif

#if defined(GPIOD)
	__HAL_RCC_GPIOD_CLK_ENABLE();
#endif

#if defined(GPIOE)
	__HAL_RCC_GPIOE_CLK_ENABLE();
#endif

#if defined(GPIOF)
	__HAL_RCC_GPIOF_CLK_ENABLE();
#endif
}

void FSKMod::initGPIOOutputLevel(void)
{
	outputPorts.dataPort->ODR &= ~(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
        						|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	outputPorts.rstPort->BRR = outputPorts.rstPin;
	outputPorts.clkPort->BRR = outputPorts.clkPin;
}

void FSKMod::initConfigGPIOPinsOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = outputPorts.rstPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(outputPorts.rstPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = outputPorts.fupPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(outputPorts.fupPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = outputPorts.clkPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(outputPorts.clkPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
		        		|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(outputPorts.dataPort, &GPIO_InitStruct);
}

void FSKMod::initConfigGPIOPinsInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = inputPorts.freqPin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(inputPorts.freqPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = inputPorts.pttPin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(inputPorts.pttPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = inputPorts.bitsPin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(inputPorts.bitsPort, &GPIO_InitStruct);
}

void FSKMod::initNVICInterrupt(void)
{
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void FSKMod::init(void)
{
	initGPIOClock();
	initGPIOOutputLevel();
	initConfigGPIOPinsOutput();
	initConfigGPIOPinsInput();
	initNVICInterrupt();

	pttState = readInitPTTState();
}

void FSKMod::delayPrimitive(uint32_t del)
{
	for(uint32_t i=0; i<del; i++)
	{
		asm("NOP");
	}
}

void FSKMod::delay(void)
{
	if(sps < 2)
	{
		delayPrimitive(1);
	}
	else if(sps < 4)
	{
		delayPrimitive(1);
	}
	else if(sps < 8)
	{
		delayPrimitive(1);
	}
	else
	{
		delayPrimitive(1);
	}
}

inline void FSKMod::writeWord(uint8_t word)
{
	outputPorts.dataPort->BSRR = ((uint32_t)word) + (~(((uint32_t)word) << 16));
	outputPorts.clkPort->ODR ^= outputPorts.clkPin;
	delay();
	outputPorts.clkPort->ODR ^= outputPorts.clkPin;
}

inline void FSKMod::updateFreq(void)
{
	outputPorts.fupPort->ODR ^= outputPorts.fupPin;
	delay();
	outputPorts.fupPort->ODR ^= outputPorts.fupPin;
}

inline bool FSKMod::readInitPTTState(void)
{
	// TODO return inverted for testing

	return !((inputPorts.pttPort->IDR & inputPorts.pttPin) != 0x00u ? 1 : 0);
}

void FSKMod::initIdleWord(void)
{
	uint32_t word = DDS_CONST * freq / DDS_CLOCK;

	for(int i = 0; i<4; i++)
	{
		idleWord[i] = ((word >> (i*8)) & 0xFF);
	}

	idleWord[4] = 0x01;
}

void FSKMod::initWord(bool setChannel)
{
	uint32_t words[MAX_SPS];

	for(int i=0; i<MAX_SPS; i++)
	{
		words[i] = DDS_CONST * (freqCh[setChannel] + (GaussianStep[i] * 4800)) / DDS_CLOCK;
		for(int j=0; j<(LOAD_CYCLE-1) ;j++)
		{
			tuningWords[setChannel][i][j] = (words[i] >> (j*8)) & 0xFF;
		}

		tuningWords[setChannel][i][4] = 0x01;
	}
}

/*
 * Public methods
 */

void FSKMod::setInputPorts(InputPorts_t inputPorts)
{
	this->inputPorts = inputPorts;

	init();
}

void FSKMod::setOutputPorts(OutputPorts_t outputPorts)
{
	this->outputPorts = outputPorts;

	reset();
}

void FSKMod::reset(void)
{
	init();

	outputPorts.fupPort->ODR &= ~(outputPorts.fupPin);
	outputPorts.clkPort->ODR &= ~(outputPorts.clkPin);

	HAL_GPIO_TogglePin(outputPorts.rstPort, outputPorts.rstPin);
	HAL_GPIO_TogglePin(outputPorts.rstPort, outputPorts.rstPin);

	outputPorts.dataPort->ODR &= ~(0xFF);

	HAL_GPIO_TogglePin(outputPorts.clkPort, outputPorts.clkPin);
	HAL_GPIO_TogglePin(outputPorts.clkPort, outputPorts.clkPin);

	HAL_GPIO_TogglePin(outputPorts.fupPort, outputPorts.fupPin);
	HAL_GPIO_TogglePin(outputPorts.fupPort, outputPorts.fupPin);
}

void FSKMod::setSamplePerSymbol(int newsps)
{
	if(newsps < 2)
	{
		sps = 1;
	}
	else if(newsps <4)
	{
		sps = 2;
	}
	else if(newsps < 8)
	{
		sps = 4;
	}
	else
	{
		sps = 8;
	}
}

void FSKMod::setPTT(bool pttState)
{
	this->pttState = pttState;
}

bool FSKMod::getPTTState(void)
{
	return pttState;
}

void FSKMod::setChannel(bool channel)
{
	this->channel = channel;
}

void FSKMod::modulateRisingEdge(void)
{
	uint8_t inc = MAX_SPS/sps;

	for(int i=0; i<MAX_SPS; i+=inc)
	{
		for(int j=0; j<5; j++)
		{
			writeWord(tuningWords[channel][i][j]);
			delay();
		}

		updateFreq();
	}
}

void FSKMod::modulateFallingEdge(void)
{
	uint8_t inc = MAX_SPS/sps;

	for(int i=0; i<MAX_SPS; i+=inc)
	{
		for(int j=0; j<5; j++)
		{
			writeWord(tuningWords[channel][MAX_SPS-1-i][j]);
			delay();
		}

		updateFreq();
	}
}

void FSKMod::idleState(void)
{
	for(int j=0; j<5; j++)
	{
		writeWord(idleWord[j]);
		delay();
	}

	updateFreq();
}

void FSKMod::setIdleFreq(uint32_t freq)
{
	this->freq = freq;

	initIdleWord();
}

void FSKMod::setChAFreq(uint32_t freqA)
{
	freqCh[CHANNEL_A] = DDS_CLOCK - freqA;

	initWord(CHANNEL_A);
}

void FSKMod::setChBFreq(uint32_t freqB)
{
	freqCh[CHANNEL_B] = DDS_CLOCK - freqB;

	initWord(CHANNEL_B);
}

FSKMod::~FSKMod() {
	// TODO Auto-generated destructor stub
}

