/*
  Vcc - A supply voltage measuring library for Arduino

  Created by Ivo Pullens, Emmission, 2014

  Inspired by:
  http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Vcc.h"

Vcc::Vcc(const float correction)
    : m_correction(correction)
{
}

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define ADMUX_VCCWRT1V1 (_BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1))
#define _IVREF 1.1
#define _ADCMAXRES 1024.0
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define ADMUX_VCCWRT1V1 (_BV(MUX5) | _BV(MUX0))
#define _IVREF 1.1
#define _ADCMAXRES 1024.0
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define ADMUX_VCCWRT1V1 (_BV(MUX3) | _BV(MUX2))
#define _IVREF 1.1
#define _ADCMAXRES 1024.0
#elif defined(__LGT8FX8P__)
#define ADMUX_VCCWRT1V1 (_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX0))
#define _IVREF 1.024
#define _ADCMAXRES 4096.0
#elif defined(__LGT8FX8E__)
#define ADMUX_VCCWRT1V1 (_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1))
#define _IVREF 1.25
#define _ADCMAXRES 4096.0
#else // defined(__AVR_ATmega328P__)
#define ADMUX_VCCWRT1V1 (_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1))
#define _IVREF 1.1
#define _ADCMAXRES 1024.0
#endif

uint16_t adcRead_(void)
{
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  return ADC;
}

float Vcc::Read_Volts(void)
{
  uint8_t adcsra_bak = ADCSRA;
  uint8_t adcsrb_bak = ADCSRB;
  uint8_t admux_bak = ADMUX;
#if defined(__LGT8FX8P__) || defined(__LGT8FX8E__)
  uint8_t adcsrd_bak = ADCSRD;
  uint8_t adcsrc_bak = ADCSRC;
#endif

  analogReference(DEFAULT);
#if defined(__LGT8FX8P__)
  ADCSRD |= _BV(BGEN);
#endif

  ADCSRA = (adcsra_bak | _BV(ADEN)) & ~_BV(ADATE);
  ADCSRB = (adcsrb_bak & ~(_BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0)));

  if (ADMUX != ADMUX_VCCWRT1V1)
  {
    ADMUX = ADMUX_VCCWRT1V1;
    delayMicroseconds(350);
  }

  (void)adcRead_();

  uint16_t pVal =
#if defined(__LGT8FX8P__)
      ({ ADCSRC |=  _BV(SPN); uint16_t nVal = adcRead_(); ADCSRC &= ~_BV(SPN);
       uint16_t p = adcRead_(); (uint16_t)((p + nVal) >> 1); });
#else
      adcRead_();
#endif

#if defined(__LGT8FX8E__)
  pVal -= (pVal >> 5);
#elif defined(__LGT8FX8P__)
  pVal -= (pVal >> 7);
#endif

  float vcc = m_correction * _IVREF * _ADCMAXRES / pVal;

#if defined(__LGT8FX8P__) || defined(__LGT8FX8E__)
  ADCSRD = adcsrd_bak;
  ADCSRC = adcsrc_bak;
#endif
  ADMUX = admux_bak;
  ADCSRB = adcsrb_bak;
  ADCSRA = adcsra_bak;

  return vcc;
}

float Vcc::Read_Perc(const float range_min, const float range_max, const boolean clip)
{
  // Read Vcc and convert to percentage
  float perc = 100.0 * (Read_Volts() - range_min) / (range_max - range_min);
  // Clip to [0..100]% range, when requested.
  if (clip)
    perc = constrain(perc, 0.0, 100.0);

  return perc;
}
