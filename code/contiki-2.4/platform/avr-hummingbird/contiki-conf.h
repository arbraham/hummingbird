/*
 * Copyright (c) 2020, M2M SIG/Yundou tech.
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
 * This file is part of the Contiki operating system.
 *
 * @(#)$$
 */

/**
 * \file
 *         Configuration for sample webbox Contiki kernel
 *
 * \author
 *         Nicolas Chang <zhbsh.zhbsh@gmail.com
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

typedef int32_t s32_t;

/*
 * MCU and clock rate
 */
#define MCU_MHZ 1
#define PLATFORM PLATFORM_AVR
#define HARWARE_REVISION HUMMINGBIRD

/* Clock ticks per second */
#define CLOCK_CONF_SECOND 125

/* COM port to be used for SLIP connection */
#define SLIP_PORT RS232_PORT_0

/* Pre-allocated memory for loadable modules heap space (in bytes)*/
#define MMEM_CONF_SIZE 256

/* Use the following address for code received via the codeprop
 * facility
 */
#define EEPROMFS_ADDR_CODEPROP 0x8000

#define CCIF
#define CLIF

#define RIMEADDR_CONF_SIZE       8
#define PACKETBUF_CONF_HDR_SIZE    0           //RF230 handles headers internally

/* 0 for IPv6, or 1 for HC1, 2 for HC01 */
#define SICSLOWPAN_CONF_COMPRESSION_IPV6 0 
#define SICSLOWPAN_CONF_COMPRESSION_HC1  1 
#define SICSLOWPAN_CONF_COMPRESSION_HC01 2

#define SICSLOWPAN_CONF_COMPRESSION       SICSLOWPAN_CONF_COMPRESSION_HC01 
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS 2
#define SICSLOWPAN_CONF_FRAG              1

/* Below will prevent fragmentation of TCP packets, undef for faster page loads, simpler wireshark captures */
//#define UIP_CONF_TCP_MSS 48

/* Fragmentation uses queuebuf.c to save packets */
#define QUEUEBUF_CONF_NUM 1
#define QUEUEBUF_CONF_REF_NUM 1

/* Logging adds 200 bytes to program size */
#define LOG_CONF_ENABLED 1

#define SICSLOWPAN_CONF_MAXAGE 5

#define UIP_CONF_LL_802154       1
#define UIP_CONF_LLH_LEN         0

#define UIP_CONF_MAX_CONNECTIONS 2
#define UIP_CONF_MAX_LISTENPORTS 2
#define UIP_CONF_UDP_CONNS       2

#define UIP_CONF_IP_FORWARD      0
#define UIP_CONF_FWCACHE_SIZE    0

#define UIP_CONF_IPV6            1
#define UIP_CONF_IPV6_CHECKS     1
#define UIP_CONF_IPV6_QUEUE_PKT  1
#define UIP_CONF_IPV6_REASSEMBLY 0
#define UIP_CONF_NETIF_MAX_ADDRESSES  3
#define UIP_CONF_ND6_MAX_PREFIXES     3
#define UIP_CONF_ND6_MAX_NEIGHBORS    4  
#define UIP_CONF_ND6_MAX_DEFROUTERS   2
#define UIP_CONF_ICMP6           1

#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1

#define UIP_CONF_TCP             1
#define UIP_CONF_TCP_SPLIT       1


typedef unsigned short clock_time_t;
typedef unsigned char u8_t;
typedef unsigned short u16_t;
typedef unsigned long u32_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

/*
 * SPI bus configuration for the Hummingbird.
 */

/* init values below */
#define SSPORT	D
#define SSPIN      (0x04)
#define SPIPORT    B
#define MOSIPIN    (0x05)
#define MISOPIN    (0x06)
#define SCKPIN     (0x07)

/* Set the default value if no defined */
#ifndef CEPIN
#define CEPIN      (0x06)
#endif 

#ifndef SSPIN
#define SSPIN      (0x04)
#endif 

#ifndef MOSIPIN
#define MOSIPIN    (0x05)
#endif

#ifndef MISOPIN
#define MISOPIN    (0x06)
#endif

#ifndef SCKPIN
#define SCKPIN     (0x07)
#endif

#ifndef SPIPORT
#define SPIPORT  B
#endif

/* For architectures that have all SPI signals on the same port */
#ifndef SSPORT
#define SSPORT SPIPORT
#endif

#ifndef SCKPORT
#define SCKPORT SPIPORT
#endif

#ifndef MOSIPORT
#define MOSIPORT SPIPORT
#endif

#ifndef MISOPORT
#define MISOPORT SPIPORT
#endif

/**
 * \name Macros used to generate read register names from platform-specific definitions of ports.
 * \brief The various CAT macros (DDR, PORT, and PIN) are used to
 * assign port/pin/DDR names to various macro variables.  The
 * variables are assigned based on the specific connections made in
 * the hardware.  For example TCCR(TICKTIMER,A) can be used in place of TCCR0A
 * if TICKTIMER is defined as 0.
 * \{
 */
#define CAT(x, y)      x##y
#define CAT2(x, y, z)  x##y##z
#define DDR(x)         CAT(DDR,  x)
#define PORT(x)        CAT(PORT, x)
#define PIN(x)         CAT(PIN,  x)
/** \} */

#define BV(bitno) _BV(bitno)
				    
#if defined (__AVR_ATmega644P__)
#define SPI_CHIP_ENABLE()    ( PORT(SSPORT) &= ~BV(CEPIN) ) /* ENABLE CEn (active low) */
#define SPI_CHIP_DISABLE()   ( PORT(SSPORT) |=  BV(CEPIN) ) /* DISABLE CEn (active low) */

#else 
#error "Wrong SPI configuration in Contiki-conf.h"
#endif

/* SPI input/output registers. */
//#define SPI_TXBUF SPDR
//#define SPI_RXBUF SPDR

/** This macro will protect the following code from interrupts.*/
#define ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )
/** This macro must always be used in conjunction with AVR_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

/* Start the SPI transaction by pulling the Slave Select low. */
#define SS_LOW()    ( PORT(SSPORT) &= ~BV(SSPIN) ) /* ENABLE CSn (active low) */

/* End the transaction by pulling the Slave Select High. */
#define SS_HIGH()   ( PORT(SSPORT) |=  BV(SSPIN) ) /* DISABLE CSn (active low) */

#define SPI_ENABLE() { \
  ENTER_CRITICAL_REGION();	  \
  SS_LOW(); /* Start the SPI transaction by pulling the Slave Select low. */

#define SPI_TRANSFER_WRITE(to_write) (SPDR = (to_write))
#define SPI_TRANSFER_WAIT() ({while ((SPSR & (1 << SPIF)) == 0) {;}}) /* gcc extension, alternative inline function */
#define SPI_TRANSFER_READ() (SPDR)

#define SPI_DISABLE() \
    SS_HIGH(); /* End the transaction by pulling the Slave Select High. */ \
    LEAVE_CRITICAL_REGION(); \
    }

#define SPI_TRANSFER(to_write) (	  \
				    SPI_TRANSFER_WRITE(to_write),	\
				    SPI_TRANSFER_WAIT(),		\
				    SPI_TRANSFER_READ() )

void clock_delay(unsigned int us2);
void clock_wait(int ms10);
void clock_set_seconds(unsigned long s);
unsigned long clock_seconds(void);

#endif /* __CONTIKI_CONF_H__ */
