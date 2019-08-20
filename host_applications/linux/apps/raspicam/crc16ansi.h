/**********************************************************************

  Copyright 2012 Percepscion Pty. Ltd. All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification requires the prior written consent of the copyright
  owner.

  The above copyright notice and this permission notice shall be 
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS 
  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN 
  ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
  SOFTWARE.
 
**********************************************************************/
#ifndef _CRC16_H_
#define _CRC16_H_

#include "stdint.h"

#define CRC16_INIT 0xffff

/**
 * Convenience wrapper for calculating a crc16 for a data block.
 * @param crc The current crc value. Should initially be given as CRC_INIT.
 * @param data pointer to the data to run the crc over.
 * @param num_bytes the size of the data.
 * @returns the corresponding crc16.
 */
uint16_t crc16 (uint16_t crc, const void *data, uint16_t num_bytes);

/*
 * Same thing, with a different polynomial
 */
uint16_t crc16_ccitt (uint16_t crc, const void *data, uint16_t num_bytes);

#endif
