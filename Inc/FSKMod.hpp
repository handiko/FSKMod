/*
 * FSKMod.hpp
 *
 *  Created on: Dec 2, 2020
 *      Author: handiko
 */

#ifndef INC_FSKMOD_HPP_
#define INC_FSKMOD_HPP_

#include "main.h"

#define RECEIVE_STATE 1
#define TRANSMIT_STATE 0

#define CHANNEL_A 0
#define CHANNEL_B 1

#define MAX_SPS 8
#define LOAD_CYCLE 5

#define DDS_CLOCK 180000000UL
#define DDS_CONST 4294967296ULL

#define LED_ON 1
#define LED_OFF 0

class FSKMod {
private:
	int sps = 1;
	bool pttState = RECEIVE_STATE;
	bool channel = CHANNEL_A;
	uint8_t tuningWords[2][MAX_SPS][LOAD_CYCLE];
	uint8_t idleWord[LOAD_CYCLE];
	const double GaussianStep[MAX_SPS] = {
			-0.499639353598468,
			-0.491423687199131,
			-0.423470689216629,
			-0.183056829730049,
			0.182996006449685,
			0.423409865936264,
			0.491362863918766,
			0.499578530318103
	};

	GPIO_TypeDef* bitsPort;
	GPIO_TypeDef* pttPort;
	GPIO_TypeDef* freqPort;

	GPIO_TypeDef* dataPort;
	GPIO_TypeDef* rstPort;
	GPIO_TypeDef* fupPort;
	GPIO_TypeDef* clkPort;

	uint16_t bitsPin;
	uint16_t pttPin;
	uint16_t freqPin;

	uint16_t dataPin;
	uint16_t rstPin;
	uint16_t fupPin;
	uint16_t clkPin;

	GPIO_TypeDef* ledTxPort;
	GPIO_TypeDef* ledDataPort;
	GPIO_TypeDef* ledChaPort;
	GPIO_TypeDef* ledChbPort;

	uint16_t ledTxPin;
	uint16_t ledDataPin;
	uint16_t ledChaPin;
	uint16_t ledChbPin;

	uint32_t freq = 1000000UL;
	uint32_t freqCh[2] = {
			(DDS_CLOCK - 161975000UL),
			(DDS_CLOCK - 161025000UL)
	};

	void initGPIOClock(void);
	void initGPIOOutputLevel(void);
	void initConfigGPIOPinsOutput(void);
	void initConfigGPIOPinsInput(void);
	//void initNVICInterrupt(void);
	void init(void);
	void delayPrimitive(uint32_t del);
	void delay(void);
	inline void writeWord(uint8_t word);
	inline void updateFreq();
	inline bool readInitPTTState(void);

	void initIdleWord(void);
	void initWord(bool setChannel);

	void setLed(GPIO_TypeDef* ledPort, uint16_t ledPin, bool LED_STATE);
	void setChaLed(bool LED_STATE);
	void setChbLed(bool LED_STATE);

public:
	FSKMod();

	void setInputPort(GPIO_TypeDef* bitsPort, GPIO_TypeDef* pttPort, GPIO_TypeDef* freqPort);
	void setOutputPort(GPIO_TypeDef* dataPort, GPIO_TypeDef* rstPort, GPIO_TypeDef* fupPort, GPIO_TypeDef* clkPort);
	void setInputPin(uint16_t bitsPin, uint16_t pttPin, uint16_t freqPin);
	void setOutputPin(uint16_t dataPin, uint16_t rstPin, uint16_t fupPin, uint16_t clkPin);
	void setLEDPort(GPIO_TypeDef* ledTxPort, GPIO_TypeDef* ledDataPort, GPIO_TypeDef* ledChaPort, GPIO_TypeDef* ledChbPort);
	void setLEDPin(uint16_t ledTxPin, uint16_t ledDataPin, uint16_t ledChaPin, uint16_t ledChbPin);

	void reset(void);

	void setSamplePerSymbol(int newsps);
	void setPTT(bool pttState);
	bool getPTTState(void);
	void setChannel(bool channel);
	bool getChannel(void);
	void modulateRisingEdge(void);
	void modulateFallingEdge(void);
	void idleState(void);

	void setIdleFreq(uint32_t freq);
	void setChAFreq(uint32_t freqA);
	void setChBFreq(uint32_t freqB);

	void setTxLed(void);
	void setDataLed(bool LED_STATE);
	void setChannelLed(void);

	virtual ~FSKMod();
};

#endif /* INC_G071RB_FSKMOD_HPP_ */
