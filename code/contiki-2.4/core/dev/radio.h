/**
 * \addtogroup dev
 * @{
 */

/**
 * \defgroup radio Radio API
 *
 * The radio API module defines a set of functions that a radio device
 * driver must implement.
 *
 * @{
 */

/*
 * Copyright (c) 2005, Swedish Institute of Computer Science.
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
 * $Id: radio.h,v 1.6 2009/11/13 08:59:22 fros4943 Exp $
 */

/**
 * \file
 *         Header file for the radio API
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#ifndef __RADIO_H__
#define __RADIO_H__

/**
 * The structure of a device driver for a radio in Contiki.
 */
struct radio_driver {
  /** Send a packet */
  int (* send)(const void *payload, unsigned short payload_len);

  /** Read a received packet into a buffer. */
  int (* read)(void *buf, unsigned short buf_len);

  /** Set a function to be called when a packet has been received. */
  void (* set_receive_function)(void (*f)(const struct radio_driver *d));

  /** Turn the radio on. */
  int (* on)(void);

  /** Turn the radio off. */
  int (* off)(void);
};

/* Generic radio return values. */
enum {
  RADIO_TX_OK,
  RADIO_TX_ERR,
  RADIO_TX_COLLISION,
  RADIO_TX_NOACK,
};

#endif /* __RADIO_H__ */

/** @} */
/** @} */
