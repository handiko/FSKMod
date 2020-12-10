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

struct InputPorts_t {
	GPIO_TypeDef* bitsPort;
	GPIO_TypeDef* pttPort;
	GPIO_TypeDef* freqPort;

	uint16_t bitsPin;
	uint16_t pttPin;
	uint16_t freqPin;
};

struct OutputPorts_t {
	GPIO_TypeDef* dataPort;
	GPIO_TypeDef* rstPort;
	GPIO_TypeDef* fupPort;
	GPIO_TypeDef* clkPort;

	uint16_t dataPin;
	uint16_t rstPin;
	uint16_t fupPin;
	uint16_t clkPin;
};

// TODO not implemented yet
struct LedPorts_t {
	GPIO_TypeDef* ledTxPort;
	GPIO_TypeDef* ledDataPort;
	GPIO_TypeDef* ledCHAPort;
	GPIO_TypeDef* ledCHBPort;

	uint16_t ledTxPin;
	uint16_t ledDataPin;
	uint16_t ledCHAPin;
	uint16_t ledCHBPin;
};

struct IRQs_t {
	IRQn_Type bitsIRQ;
	IRQn_Type pttIRQ;
};

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

	InputPorts_t inputPorts;
	OutputPorts_t outputPorts;

	IRQs_t interrupts;

	uint32_t freq = 1000000UL;
	uint32_t freqCh[2] = {
			(DDS_CLOCK - 161975000UL),
			(DDS_CLOCK - 161025000UL)
	};

	void initGPIOClock(void);
	void initGPIOOutputLevel(void);
	void initConfigGPIOPinsOutput(void);
	void initConfigGPIOPinsInput(void);
	void initNVICInterrupt(void);
	void init(void);
	void delayPrimitive(uint32_t del);
	void delay(void);
	inline void writeWord(uint8_t word);
	inline void updateFreq();
	inline bool readInitPTTState(void);

	void initIdleWord(void);
	void initWord(bool setChannel);

public:
	FSKMod();

	void setInputPorts(InputPorts_t inputPorts);
	void setOutputPorts(OutputPorts_t outputPorts);
	void setInterruptRequest(IRQs_t interrupts);

	void reset(void);

	void setSamplePerSymbol(int newsps);
	void setPTT(bool pttState);
	bool getPTTState(void);
	void setChannel(bool channel);
	void modulateRisingEdge(void);
	void modulateFallingEdge(void);
	void idleState(void);

	void setIdleFreq(uint32_t freq);
	void setChAFreq(uint32_t freqA);
	void setChBFreq(uint32_t freqB);

	virtual ~FSKMod();
};

#endif /* INC_G071RB_FSKMOD_HPP_ */
