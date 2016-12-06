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

#ifndef UART_H_
#define UART_H_

void uart_print(char *str);
void uart_putc(char c);
int uart_getc(void);
void uart_init(int rxd_pin, int txd_pin, int cts_pin, int rts_pin, int baudrate, uint8_t *rxuff, int rxbsize, uint8_t *txbuff, int txbsize);

#endif /* UART_H_ */
