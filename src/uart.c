/*
 * LPC824 UART support library
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

#include "chip.h"

/* Ring Buffers for transfer */
static RINGBUFF_T rxring;
static RINGBUFF_T txring;

/* Interrupt Handler (fixed name) */
void UART0_IRQHandler(void)
{
	Chip_UART_IRQRBHandler(LPC_USART0, &rxring, &txring);
}

/**
 * @brief	write string into tx ring buffer
 * @param	str		: pointer to the string (must be terminated by '\0')
 * @return	none
 */
void uart_print(char *str) {
	int n;
	char *p;

	p = str;
	n = 0;
	while(*p != 0) {
		p++;
		n++;
	}
	Chip_UART_SendRB(LPC_USART0, &txring, str, n);
}

/**
 * @brief	write byte into tx ring buffer
 * @param	c		: data to send
 * @return	none
 */
void uart_putc(char c) {
	Chip_UART_SendRB(LPC_USART0, &txring, &c, 1);
}

/**
 * @brief	read byte from rx ring buffer
 * @param	none
 * @return	received data, -1 if no data
 */
int uart_getc() {
	uint8_t ch;
	int n_data;

	n_data = Chip_UART_ReadRB(LPC_USART0, &rxring, &ch, 1); // read 1byte
	if(n_data == 0)
		return -1;
	else
		return ch;
}

/**
 * @brief	Initialize UART
 * @param	rxd_pin		: pin no for RXD (i.e. 4 for P0_4)
 * @param	txd_pin		: pin no for TXD
 * @param	baudrate	: baudrate in integer
 * @param	rxbuff		: ring buffer to receive data
 * @param	rxbsize		: buffer size of rxbuff in byte count (must be power of 2)
 * @param	txbuff		: ring buffer to send data
 * @param	txbsize		: buffer size of txbuff in byte count (must be power of 2)
 */
void uart_init(int rxd_pin, int txd_pin, int cts_pin, int rts_pin,
				int baudrate,
				uint8_t *rxbuff, int rxbsize, uint8_t *txbuff, int txbsize) {

	/* Init pin MUX */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_Clock_SetUARTClockDiv(1);	/* divided by 1 */
	Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, rxd_pin);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, txd_pin);
	if(cts_pin >= 0)
		Chip_SWM_MovablePinAssign(SWM_U0_CTS_I, cts_pin);
	if(rts_pin >= 0)
		Chip_SWM_MovablePinAssign(SWM_U0_RTS_O, rts_pin);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Init UART */
	Chip_UART_Init(LPC_USART0);
	if(cts_pin >= 0)
		Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1 | UART_CFG_CTSEN);
	else
		Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART0, baudrate);
	Chip_UART_Enable(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);

	/* Init ring buffer */
	RingBuffer_Init(&rxring, rxbuff, 1, rxbsize);
	RingBuffer_Init(&txring, txbuff, 1, txbsize);

	/* Enable Interrupt  */
	Chip_UART_IntEnable(LPC_USART0, UART_INTEN_RXRDY);
	Chip_UART_IntEnable(LPC_USART0, UART_INTEN_TXRDY);
	NVIC_SetPriority(UART0_IRQn, 1);
	NVIC_EnableIRQ(UART0_IRQn);
}

