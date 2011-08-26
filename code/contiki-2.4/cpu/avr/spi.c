/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 3. Neither the name of the Institute nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE. 
 *
 * @(#)$Id: spi.c,v 1.1 2007/01/25 18:22:55 bg- Exp $
 */

#include <avr/io.h>

#include "contiki-conf.h"

/*
 * On the Tmote sky access to I2C/SPI/UART0 must always be
 * exclusive. Set spi_busy so that interrupt handlers can check if
 * they are allowed to use the bus or not. Only the CC2420 radio needs
 * this in practice.
 * 
 */
unsigned char spi_busy = 0;

/*
 * Initialize SPI bus.
 */
void
spi_init(void)
{
  static unsigned char spi_inited = 0;

  if (spi_inited)
    return;

  /* Initalize ports for communication with SPI units. */
  /* CSN=SS and must be output when master! */  
  #if defined (__AVR_ATmega644P__)
  /*SPI Specific Initialization.*/
  /* Set SS, CLK and MOSI as output. */
  /* 
    * The DDxn bit in the DDRx Register selects the direction of this pin. If DDxn is written logic one,
    * Pxn is configured as an output pin. If DDxn is written logic zero, Pxn is configured as an input
    * pin.
  */
  DDR(SSPORT) |= BV(SSPIN);  
  DDR(SPIPORT) |= BV(SCKPIN) | BV(MOSIPIN);

  DDR(SSPORT) |= BV(0x6);  //chip enable, need remove this chip enable pin in the HW

  /* If PORTxn is written logic one when the pin is configured as an output pin, the port pin is driven
    * high (one). If PORTxn is written logic zero when the pin is configured as an output pin, the port
    * pin is driven low (zero).
 */
  PORT(SPIPORT) |= BV(SCKPIN) | BV(MOSIPIN);		//driven to high
  PORT(SSPORT) |= BV(SSPIN); 		//driven to high

  PORT(SSPORT) |= BV(0x6);		//chip enable, need remove this chip enable pin in the HW
  
#else
#error "No CPU select, spi_init() failed in spi.c"
#endif

  /* Enables SPI, selects "master", clock rate FCK / 2, and SPI mode 0 */
  SPCR = BV(SPE) | BV(MSTR);
  SPSR = BV(SPI2X);
}
