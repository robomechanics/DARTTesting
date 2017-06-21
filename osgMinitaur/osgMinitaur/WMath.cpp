/**
 * Part of the Wiring project - http://wiring.org.co
  Copyright (c) 2004-06 Hernando Barragan
  Modified by David A. Mellis for Arduino - http://www.arduino.cc/, Avik De <avikde@gmail.com>

  This file is part of koduino <https://github.com/avikde/koduino>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {
  #include "stdlib.h"
  #include "stdint.h"
}
#include "WMath.h"
#include "wiring_constants.h"
//#include "arm_math.h"

extern void randomSeed( uint32_t dwSeed )
{
  if ( dwSeed != 0 )
  {
    srand( dwSeed ) ;
  }
}

extern long random( long howbig )
{
  if ( howbig == 0 )
  {
    return 0 ;
  }

  return rand() % howbig;
}

extern long random( long howsmall, long howbig )
{
  if (howsmall >= howbig)
  {
    return howsmall;
  }

  long diff = howbig - howsmall;

  return random(diff) + howsmall;
}

// Be aware of http://forum.arduino.cc/index.php?topic=46546.msg336462#msg336462
extern long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern uint16_t makeWord( uint16_t w )
{
  return w ;
}

extern uint16_t makeWord( uint8_t h, uint8_t l )
{
  return (h << 8) | l ;
}

// NEW


extern float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern float map(uint32_t x, uint32_t in_min, uint32_t in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern float map(float x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern float interp1(float from, float to, float frac)
{
  return (1-frac) * from + frac * to;
}
// fraction for keyframing
extern float interpFrac(uint32_t startTime, uint32_t endTime, uint32_t now)
{
  return (now - startTime)/((float)(endTime - startTime));
}

//
//

float DLPF::update(float val) {
  float delta;
  switch (type)
  {
    case DLPF_ANGRATE:
      delta = fmodf_mpi_pi(val - oldVal);
      vel = interp1(delta * freq, vel, smooth);
      oldVal = val;
      return vel;
    case DLPF_RATE:
      delta = val - oldVal;
      vel = interp1(delta * freq, vel, smooth);
      oldVal = val;
      return vel;
    case DLPF_INTEGRATE:
      val = smooth * oldVal + val / freq;
      return val;
    case DLPF_SMOOTH:
    default:
      oldVal = interp1(val, oldVal, smooth);
      return oldVal;
  }
}

