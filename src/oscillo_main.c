/*
 * BT Oscillo
 *
 * Copyright (C) 2016 Masayoshi Tanaka @ Workshop Sei-Ryu-An
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #if defined(NO_BOARD_LIB)
 #include "chip.h"
 #else
 #include "board.h"
 #endif

#include "uart.h"

// data buffer for UART interrupt handler
#define UART_BUFSIZE_RX 32
uint8_t uartBuffRx[UART_BUFSIZE_RX];
#define UART_BUFSIZE_TX 1024
uint8_t uartBuffTx[UART_BUFSIZE_TX];

/* SysTick Flag (set 1 by SysTick Handler) */
static volatile int tic_f = 0;

/* SysTick Handler (Frame synchronize) */
void SysTick_Handler(void) {
	tic_f = 1;
}

/* ADC Setup Info */
static bool sequenceComplete;
static bool thresholdCrossed;
/* CH1 for 824MAX, CH2 for original board */
#define BOARD_ADC_CH 1

/* ADC default clock (clock = 25*SamplingRate) */
int adc_clock = 1200000; //1.2MHz = 48k sample/sec

/* ADC Data Buffer */
#define NFRAME 480 // number of sample per 1frame
uint16_t adc_buff[NFRAME];

/* Data transfer mode */
#define XFER_BURST 0
#define XFER_CONT 1
int transfer_mode = XFER_BURST;
int burst_length = NFRAME;
int frame_count = 0;

/* Trigger Mode */
#define TRIGGER_FREE 0
#define TRIGGER_POSITIVE 1
#define TRIGGER_NEGATIVE 2
int trigger_mode = TRIGGER_FREE;
uint32_t trigger_level = 512<<6; //Level (0-1023 , 512=center)<<6 (bit shift for DR-register field)
#define WAIT_TRIGGER_LIMIT 100000 //Time out
#define WAIT_TRIGGER_OK 1
#define WAIT_TRIGGER_ERR 0

/* Run State */
#define STATE_STOP 0
#define STATE_RUN 1
#define STATE_SINGLE 2
int state = STATE_RUN;

/*
 *  ADC Interrupt Handler
 */
void ADC_SEQA_IRQHandler(void)
{
	uint32_t pending;

	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC);

	/* Sequence A completion interrupt */
	if (pending & ADC_FLAGS_SEQA_INT_MASK) {
		sequenceComplete = true;
	}

	/* Threshold crossing interrupt on ADC input channel */
	if (pending & ADC_FLAGS_THCMP_MASK(BOARD_ADC_CH)) {
		thresholdCrossed = true;
	}

	/* Clear any pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}

/*
 *  ADC Initialization
 */
void adc_init() {

	// Reset & Calibration
	Chip_ADC_Init(LPC_ADC, 0);	//12-bit mode and normal power
	Chip_ADC_StartCalibration(LPC_ADC);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC))) {}
	Chip_ADC_SetClockRate(LPC_ADC, adc_clock);

	// Setup sequencer for ADC2(20pin)
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX,
							(ADC_SEQ_CTRL_CHANSEL(BOARD_ADC_CH) | ADC_SEQ_CTRL_MODE_EOS));
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0+BOARD_ADC_CH);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	// Setup threshold
	Chip_ADC_SetThrLowValue(LPC_ADC, 0, 0);
	Chip_ADC_SetThrHighValue(LPC_ADC, 0, 0xFFF);
	Chip_ADC_SelectTH0Channels(LPC_ADC, ADC_THRSEL_CHAN_SEL_THR1(BOARD_ADC_CH));
	//Chip_ADC_SetThresholdInt(LPC_ADC, BOARD_ADC_CH, ADC_INTEN_THCMP_CROSSING);

	// Setup Interrupt
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
	NVIC_EnableIRQ(ADC_SEQA_IRQn);

	// Enable sequencer
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);
}

/*
 * Start Burst Sampling
 */
void adc_start() {
	Chip_ADC_SetClockRate(LPC_ADC, adc_clock);
	Chip_ADC_StartBurstSequencer(LPC_ADC, ADC_SEQA_IDX);
}

/*
 * Stop Burst Sampling
 */
void adc_stop() {
	Chip_ADC_StopBurstSequencer(LPC_ADC, ADC_SEQA_IDX);
}

/*
 *　Trigger process
 */
int wait_trigger() {
	uint32_t raw_sample = 0;
	int data_valid;
	uint32_t adc_new; // newest ADC value
	uint32_t adc_old; // previous ADC value
	int count; // counter for time out

	if(trigger_mode == TRIGGER_FREE) {
		return WAIT_TRIGGER_OK; //free->Always OK
	}
	else if(trigger_mode == TRIGGER_POSITIVE) {
		data_valid = 0;
		while(data_valid == 0) {
			raw_sample = LPC_ADC->DR[BOARD_ADC_CH];
			data_valid = raw_sample & ADC_SEQ_GDAT_DATAVALID;
		}
		adc_new = raw_sample & 0xffc0;
		count = 0;
		while(count < WAIT_TRIGGER_LIMIT) {
			data_valid = 0;
			while(data_valid == 0) {
				raw_sample = LPC_ADC->DR[BOARD_ADC_CH];
				data_valid = raw_sample & ADC_SEQ_GDAT_DATAVALID;
			}
			adc_old = adc_new;
			adc_new = raw_sample & 0xffc0;
			if((adc_new >= trigger_level)&&(adc_old < trigger_level)) {
				return WAIT_TRIGGER_OK; //Wait OK
			}
			count++;
		}
	}
	else if(trigger_mode == TRIGGER_NEGATIVE){
		data_valid = 0;
		while(data_valid == 0) {
			raw_sample = LPC_ADC->DR[BOARD_ADC_CH];
			data_valid = raw_sample & ADC_SEQ_GDAT_DATAVALID;
		}
		adc_new = raw_sample & 0xffc0;
		count = 0;
		while(count < WAIT_TRIGGER_LIMIT) {
			data_valid = 0;
			while(data_valid == 0) {
				raw_sample = LPC_ADC->DR[BOARD_ADC_CH];
				data_valid = raw_sample & ADC_SEQ_GDAT_DATAVALID;
			}
			adc_old = adc_new;
			adc_new = raw_sample & 0xffc0;
			if((adc_new <= trigger_level)&&(adc_old > trigger_level))  {
				return WAIT_TRIGGER_OK; //Wait OK
			}
			count++;
		}
	}
	return WAIT_TRIGGER_ERR; //Error(time out)
}

/*
 * Read ADC data to buffer (NFRAME byte)
 */
void read_block() {
	int i;
	uint32_t raw_sample = 0;
	int data_valid;

	for(i = 0; i < NFRAME; i++) {
		data_valid = 0;
		while(data_valid == 0) {
			raw_sample = LPC_ADC->DR[BOARD_ADC_CH];
			data_valid = raw_sample & ADC_SEQ_GDAT_DATAVALID;
		}
		adc_buff[i] = (uint16_t)raw_sample;
	}

	// bit shift
	for(i = 0; i < NFRAME; i++) {
		adc_buff[i] = adc_buff[i]>>6;
	}
}

/*
 * Read single ADC data
 */
uint16_t read_sample() {
	uint32_t raw_sample = 0;
	int data_valid;

	data_valid = 0;
	while(data_valid == 0) {
		raw_sample = LPC_ADC->DR[BOARD_ADC_CH];
		data_valid = raw_sample & ADC_SEQ_GDAT_DATAVALID;
	}
	return (raw_sample&0xffc0)>>6;
}

/*
 * Workaround for RN42's bug
 */
void wait1ms() {
	volatile int c;

	// Software Loop
	for(c = 0; c < 15000; c++) {}

	// Wait after Buffer Empty
	//while((LPC_USART->LSR & UART_LSR_TEMT)==0) {}
	//for(c = 0; c < 1500; c++) {}
}

/*
 * Send buffer data to Host (in Burst-Mode)
 */
void send_block() {
	int i;
	uint8_t data;

	for(i = 0; i < burst_length; i++) {
		/* Upper 5bit (bit5 = 0) */
		data = (adc_buff[i]>>5)&0x1f;
		uart_putc(data);
		/* Lower 5bit (bit5 = 1) */
		data = (adc_buff[i]&0x1f);
		if(i == burst_length-1) {
			data |= 0x40; //Add FlagSet(Burst-EOR)
		}
		else {
			data |= 0x20; //Add FlagSet(Burst-Low)
		}
		uart_putc(data);
		//if((i < burst_length-50)&&((i%50) == 49)) wait1ms(); // interval for every 100Bytes (workaround/RN42-bug)
	}
}

/*
 * Send single data (in Continuous-Mode)
 */
void send_sample(uint16_t sample, int eor) {
	int data;

	/* Upper 5bit (flagset = 000) */
	data = (sample>>5) & 0x1f;
	uart_putc(data);

	/* Lower 5bit */
	data = sample & 0x1f;
	if(eor == 1) {
		data |= 0xa0; //Add-FlagSet(Continuous-EOR)
	}
	else {
		data |= 0x60; //Add FlagSet(Continuous-Low)
	}
	uart_putc(data);
}

/*
 * Send force discard command  to Host
 */
void send_discard() {
	uart_putc(0xc0);
}

/*
 * Switch to continuous mode
 */
void set_continuous_mode( char *buff) {
	int rate;

	switch(*buff) {
	case '1':
		rate = 24;//24Hz=2sec/div
		break;
	case '2':
		rate = 48;//48Hz=1sec/div
		break;
	case '3':
		rate = 96;//96Hz=0.5sec/div
		break;
	case '4':
		rate = 480;//*not use*
		break;
	}
	adc_clock = 96000; //sampling rate for trigger checking
	transfer_mode = XFER_CONT;
	burst_length = 1;
	frame_count = 0;
	SysTick_Config(SystemCoreClock/rate);
}

/*
 * Switch to burst mode
 */
void set_burst_mode( char *buff ) {
	int rate;

	switch(*buff) {
	case '1':
		rate = 9600;//9.6kHz=5msec/div
		break;
	case '2':
		rate = 48000;//48kHz=1msec/div
		break;
	case '3':
		rate = 96000;//96kHz=0.5msec/div
		break;
	case '4':
		rate = 192000;//not use (DMA is necessary)
		break;
	}
	adc_clock = rate*25;
	transfer_mode = XFER_BURST;
	burst_length = NFRAME;
	frame_count = 0;
	SysTick_Config(SystemCoreClock/8);
}

/*
 *  Interpret command from Host
 */
void set_trigger_mode(char *buff) {
	switch(*buff) {
	case 'P':
		trigger_mode = TRIGGER_POSITIVE;
		break;
	case 'N':
		trigger_mode = TRIGGER_NEGATIVE;
		break;
	case 'F':
		trigger_mode = TRIGGER_FREE;
		break;
	}
}
void set_run_state(char *buff) {
	switch(*buff) {
	case '0':
		state = STATE_STOP;
		break;
	case '1':
		state = STATE_RUN;
		frame_count = 0;
		break;
	case '2':
		state = STATE_SINGLE;
		frame_count = 0;
		send_discard();
		break;
	}
}

void set_gpio_state(char *buff) {
	switch(*buff) {
	case '0':
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, false);
		break;
	case '1':
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, true);
		break;


	}
}

void exec_cmd(char *buff) {
	switch(*buff) {
	case 'C':
		set_continuous_mode(buff+1);
		send_discard();
		break;
	case 'B':
		set_burst_mode(buff+1);
		send_discard();
		break;
	case 'R':
		set_run_state(buff+1);
		break;
	case 'T':
		set_trigger_mode(buff+1);
		break;
	case 'G':
		set_gpio_state(buff+1);
		break;
	}
}

#define CMD_LEN 16
static char cmd_buff[CMD_LEN]; //command buffer
static char *cmd_bufp;

/*
 *  OscilloScope  Main
 */
int main(void) {
	int trig; //trigger result
	uint16_t sample_data;
	int d; //byte data from host

	/* System Init */
	SystemCoreClockUpdate();
	adc_init();
	uart_init(0, 4, 17, -1, 115200, uartBuffRx, UART_BUFSIZE_RX, uartBuffTx, UART_BUFSIZE_TX);
	set_burst_mode("1\n"); //default="B1\n"=5msec/divß

	/* SysTick Timer for block interval (1/8fps = 125msec) */
	SysTick_Config(SystemCoreClock/8);

	/* GPIO init (P0_15 for output) */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 15);

	/* Main Loop */
	cmd_bufp = cmd_buff;
	while (1) {

		/* Check command from UART */
		while((d = uart_getc()) >= 0) {
			if(d == '\n') {
				exec_cmd(cmd_buff);
				cmd_bufp = cmd_buff;
			}
			else {
				*cmd_bufp = (char)d;
				cmd_bufp++;
				if(cmd_bufp-cmd_buff > CMD_LEN) {
					cmd_bufp = cmd_buff;
				}
			}
		}

		/* wait SysTick */
		while(tic_f == 0) {};
		tic_f = 0;

		if(transfer_mode == XFER_BURST) {
			/* Burst-mode */
			adc_start();
			trig = wait_trigger();
			read_block();
			adc_stop();
			if(trig == WAIT_TRIGGER_OK) {
				if(state != STATE_STOP) {
					send_block();
				}
				if(state == STATE_SINGLE) {
					state = STATE_STOP;
				}
			}
		}
		else {
			/* Continuous mode */
			adc_start();
			if(frame_count == 0) wait_trigger();
			sample_data = read_sample();
			adc_stop();
			if(state != STATE_STOP) {
				if(frame_count >= NFRAME) {
					send_sample(sample_data, 1);
					frame_count = 0;
					if(state == STATE_SINGLE) {
						state = STATE_STOP;
					}
				}
				else {
					send_sample(sample_data, 0);
					frame_count++;
				}
			}
		}
	}
	return 1;
}
