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
	HAL_GPIO_WritePin(dataPort, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
	            	|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
					GPIO_PIN_RESET);
	HAL_GPIO_WritePin(rstPort, rstPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(fupPort, fupPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(clkPort, clkPin, GPIO_PIN_RESET);
}

void FSKMod::initConfigGPIOPinsOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = rstPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(rstPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = fupPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(fupPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = clkPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(clkPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
		        		|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	HAL_GPIO_Init(dataPort, &GPIO_InitStruct);
}

void FSKMod::initConfigGPIOPinsInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = freqPin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(freqPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = pttPin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(pttPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = bitsPin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(bitsPort, &GPIO_InitStruct);
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
	dataPort->BSRR = ((uint32_t)word) + (~(((uint32_t)word) << 16));
	clkPort->ODR ^= clkPin;
	delay();
	clkPort->ODR ^= clkPin;
}

inline void FSKMod::updateFreq(void)
{
	fupPort->ODR ^= fupPin;
	delay();
	fupPort->ODR ^= fupPin;
}

inline bool FSKMod::readInitPTTState(void)
{
	// TODO return inverted for testing

	return !((pttPort->IDR & pttPin) != 0x00u ? 1 : 0);
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
	//this->inputPorts = inputPorts;
	this->bitsPort = inputPorts.bitsPort;
	this->pttPort = inputPorts.pttPort;
	this->freqPort = inputPorts.freqPort;

	this->bitsPin = inputPorts.bitsPin;
	this->pttPin = inputPorts.pttPin;
	this->freqPin = inputPorts.freqPin;

	init();
}

void FSKMod::setOutputPorts(OutputPorts_t outputPorts)
{
	//this->outputPorts = outputPorts;
	this->dataPort = outputPorts.dataPort;
	this->rstPort = outputPorts.rstPort;
	this->fupPort = outputPorts.fupPort;
	this->clkPort = outputPorts.clkPort;

	this->rstPin = outputPorts.rstPin;
	this->fupPin = outputPorts.fupPin;
	this->clkPin = outputPorts.clkPin;

	reset();
}

/*
void FSKMod::setInputPort(GPIO_TypeDef* bitsPort, GPIO_TypeDef* pttPort, GPIO_TypeDef* freqPort)
{
	this->bitsPort = bitsPort;
	this->pttPort = pttPort;
	this->freqPort = freqPort;

	reset();
}

void FSKMod::setOutputPort(GPIO_TypeDef* dataPort, GPIO_TypeDef* rstPort, GPIO_TypeDef* fupPort, GPIO_TypeDef* clkPort)
{
	this->dataPort = dataPort;
	this->rstPort = rstPort;
	this->fupPort = fupPort;
	this->clkPort = clkPort;

	reset();
}

void FSKMod::setInputPin(uint16_t bitsPin, uint16_t pttPin, uint16_t freqPin)
{
	this->bitsPin = bitsPin;
	this->pttPin = pttPin;
	this->freqPin = freqPin;

	reset();
}

void FSKMod::setOutputPin(uint16_t dataPin, uint16_t rstPin, uint16_t fupPin, uint16_t clkPin)
{
	this->dataPin = dataPin;
	this->rstPin = rstPin;
	this->fupPin = fupPin;
	this->clkPin = clkPin;

	reset();
}
*/

void FSKMod::reset(void)
{
	init();

	fupPort->ODR &= ~(fupPin);
	clkPort->ODR &= ~(clkPin);

	HAL_GPIO_TogglePin(rstPort, rstPin);
	HAL_GPIO_TogglePin(rstPort, rstPin);

	dataPort->ODR &= ~(0xFF);

	HAL_GPIO_TogglePin(clkPort, clkPin);
	HAL_GPIO_TogglePin(clkPort, clkPin);

	HAL_GPIO_TogglePin(fupPort, fupPin);
	HAL_GPIO_TogglePin(fupPort, fupPin);
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

