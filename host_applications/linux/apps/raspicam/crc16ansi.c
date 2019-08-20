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
#include "crc16ansi.h"

static uint16_t crc16_update(uint16_t crc, uint8_t val, uint16_t poly)
{
  uint8_t i;
  crc = (uint16_t)(crc ^ val);
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ poly;
    else
      crc = (crc >> 1);
  }
  return crc;
}

uint16_t crc16 (uint16_t crc, const void *data, uint16_t num_bytes)
{
  const uint8_t *p = (const uint8_t *)data;
  while (num_bytes--)
    crc = crc16_update (crc, *p++, 0xa001);
  return crc;
}

uint16_t crc16_ccitt (uint16_t crc, const void *data, uint16_t num_bytes)
{
  const uint8_t *p = (const uint8_t *)data;
  while (num_bytes--)
    crc = crc16_update (crc, *p++, 0x8408);
  return crc^0xffff;
}
