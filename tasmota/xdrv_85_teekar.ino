/*
  xdrv_85_teekar.ino - TEEKAR dimmer support for Tasmota

  Author: Constantine Safronov
  The driver supports its own FADE feature as tasmota's 
  fade works only with PWM dimmers. To enable it:
  1. Go to Tasmota Console and enter commands:
  2. fade 0
  3. speed X
  Thats it.
  The Speed command has the same meaning as stated in Tasmota docs
  https://tasmota.github.io/docs/Commands/
  Pay attention, that it is needed to disable fade!

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_LIGHT
#ifdef USE_TEEKAR

#include <Wire.h>

/*********************************************************************************************\
 * Teekar dimmer
 *
 *  0  1  2  3  4  5  6  7  8  9  10
 * 65 AA                                              - Header
 *       00 01                                        - Version?
 *             05                                     - Following data length (5 bytes)
 *                00 00 00                            - Not used
 *                         XX                         - Dimmer percentage (01 to 64 = 1 to 100%)
 *                             00                     - Not used
 *                                YY                  - CRC = sum of bytes modulo 256
\*********************************************************************************************/
#define HLS028_ADDR               0x56
#define XDRV_85                   85

#define DIM_MIN 66   //dim min
#define DIM_MAX 178  //dim max
#define TEEKAR_DIM_LOOP 0.05 //Seconds

struct TEEKAR {
  uint8_t power = 255;      // Not initialized
  uint8_t dimmer = 255;     // Not initialized
  uint8_t led = 0;          // Not initialized
  float dim_curr = 0;       // Not initialized
  float dim_targ = 0;       // Not initialized
  float dim_incr = 0;       // Not initialized
} Teekar;

/********************************************************************************************/
void TeekarInit()
{
    //define pin modes for "SSPI" led control
    pinMode(Pin(GPIO_SSPI_SCLK), OUTPUT);
    digitalWrite(Pin(GPIO_SSPI_SCLK), LOW);
    pinMode(Pin(GPIO_SSPI_MOSI), OUTPUT);
}

void TeekarWriteLed(uint8_t led)
{
  if(led > 8) led = 8;
  uint8_t tmp = 0xFF;

  digitalWrite(Pin(GPIO_SSPI_SCLK), LOW);
  if(led < 5)
  {
    tmp = tmp<<led;
    for (int i=7; i>=0; i--) {
      digitalWrite(Pin(GPIO_SSPI_MOSI), tmp & (1<<i));
      digitalWrite(Pin(GPIO_SSPI_SCLK), HIGH);
      delayMicroseconds(1);
      digitalWrite(Pin(GPIO_SSPI_SCLK), LOW);
    }
  }
  else
  {
    tmp = (tmp>>(led-5))&0xE0;
    for (int i=7; i>=0; i--) {
      digitalWrite(Pin(GPIO_SSPI_SCLK), HIGH);
      digitalWrite(Pin(GPIO_SSPI_MOSI), tmp & (1<<i));
      digitalWrite(Pin(GPIO_SSPI_SCLK), LOW);
    }
  }
  digitalWrite(Pin(GPIO_SSPI_MOSI), LOW);
}

uint16_t TeekarReadTouch()
{
  uint16_t value;
  uint8_t tmp1, tmp2;

  Wire.beginTransmission(HLS028_ADDR);
  Wire.write(0x34); // reg address
  Wire.endTransmission(false);

  Wire.requestFrom(HLS028_ADDR, 1);
  tmp1 = Wire.read();

  Wire.beginTransmission(HLS028_ADDR);
  Wire.write(0x35); // reg address
  Wire.endTransmission(false);

  Wire.requestFrom(HLS028_ADDR, 1);
  tmp2 = Wire.read();
  value = (tmp1<<8) + tmp2;
  return value;
}

bool TeekarCalcDimmer()
{
  Teekar.dim_incr = 100.0 / ((Settings.light_speed * 0.5) / TEEKAR_DIM_LOOP + 1);
  //Going UP
  if(Teekar.dim_targ > Teekar.dim_curr && ((Teekar.dim_targ - Teekar.dim_curr) > Teekar.dim_incr))
  {
      Teekar.dim_curr += Teekar.dim_incr;
      return true;
  }
  //Going Down
  else if (Teekar.dim_targ < Teekar.dim_curr  && ((Teekar.dim_curr - Teekar.dim_targ) > Teekar.dim_incr))
  {
      Teekar.dim_curr -= Teekar.dim_incr;
      return true;
  }
  //Stop
  return false;
}

void TeekarSend()
{
  //                        0    1    2    3    4    5    6    7    8    9   10
  uint8_t buffer[11] = { 0x65,0xAA,0x00,0x01,0x05,0x00,0x00,0x00,0xFF,0x00,0x00 };

  buffer[8] = Teekar.dimmer;

    
  uint16_t crc = 0;
  for (uint32_t i = 0; i < sizeof(buffer); i++) {
    if (i < sizeof(buffer) - 1) {  crc+= buffer[i]; } else {buffer[10] = crc % 256;}
    Serial.write(buffer[i]);
  }
}

bool TeekarSendPower()
{
  uint8_t action = XdrvMailbox.index &1;
  if (action != Teekar.power) {
    Teekar.power = action;

    if(Teekar.power)
    {
      Teekar.dim_targ = LightGetDimmer(1);
    }
    else
    {
      if (PinUsed(GPIO_SSPI_MOSI) && PinUsed(GPIO_SSPI_SCLK))
      {
        TeekarWriteLed(0);
      } 
      Teekar.dim_targ = 0;
    }
  }
  return true;
}

bool TeekarSendDimmer(void)
{
  uint8_t dimmer = (uint8_t)Teekar.dim_curr;

  dimmer = (dimmer < Settings.dimmer_hw_min) ? Settings.dimmer_hw_min : dimmer;
  dimmer = (dimmer > Settings.dimmer_hw_max) ? Settings.dimmer_hw_max : dimmer;

  uint8_t dimc = (dimmer * (DIM_MAX - DIM_MIN))/100 + DIM_MIN;
  if (dimc != Teekar.dimmer) 
  {
    dimc <= 68 ? Teekar.dimmer = 0 : Teekar.dimmer = dimc;
    TeekarSend();
  }

  if (PinUsed(GPIO_SSPI_MOSI) && PinUsed(GPIO_SSPI_SCLK))
  {
    uint8_t led = dimmer/14+1;
    if (dimc <= 68) led = 0;
    TeekarWriteLed(led);
  }
  return true;
}

bool TeekarModuleSelected(void)
{
  SetSerial(9600, TS_SERIAL_8N1);
  TasmotaGlobal.devices_present++;
  TasmotaGlobal.light_type = LT_PWM1;//LT_SERIAL1;
  return true;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
static uint8_t tmp = 0;
bool Xdrv85(uint8_t function)
{
  bool result = false;
  uint16_t val = 0;
  uint8_t led = 0;
  if (TEEKAR == TasmotaGlobal.module_type) {
    switch (function) {
      case FUNC_EVERY_50_MSECOND:
        val = TeekarReadTouch();      
        switch(val)
        {
          case 1:
            led = 8;
          break;
          case 512:
            led = 1;
          break;
          case 1024:
            led = 2;
          break;
          case 2048:
            led = 3;
          break;  
          case 4096:
            led = 4;
          break;  
          case 8192:
            led = 5;
          break;  
          case 16384:
            led = 6;
          break; 
          case 32768:
            led = 7;
          break;                                              
        }
        if(led && Teekar.power)
        {
          if (Teekar.led != led && PinUsed(GPIO_SSPI_MOSI) && PinUsed(GPIO_SSPI_SCLK))
          {
            Teekar.led = led;
            //TeekarWriteLed(led);
            char scmnd[20];
            uint8_t val;
            if(led==8)
              val = 100;
            else
              val = led*12;
            
            snprintf_P(scmnd, sizeof(scmnd), PSTR(D_CMND_DIMMER " %d"), val);
            ExecuteCommand(scmnd, SRC_SWITCH);
          }
        }

        if(TeekarCalcDimmer())
        {
          TeekarSendDimmer();
        }
        break;
      case FUNC_SET_DEVICE_POWER:
        result = TeekarSendPower();
        break;
      case FUNC_SET_CHANNELS:
        Teekar.dim_targ = LightGetDimmer(1);
        break;
      case FUNC_MODULE_INIT:
        TeekarInit(); 
        result = TeekarModuleSelected();
        break;
    }
  }
  return result;
}

#endif  // USE_TEEKAR
#endif  // USE_LIGHT
