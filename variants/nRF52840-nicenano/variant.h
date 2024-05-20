/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_NICE_NANO_
#define _VARIANT_NICE_NANO_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

#define USE_LFXO      // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define _PINNUM(port, pin)     ((port)*32 + (pin))

// Number of pins defined in PinDescription array
#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED1             (15) // Red led
#define LED_BUILTIN          PIN_LED1  
#define LED_CONN             PIN_LED1  

#define LED_RED              PIN_LED1  
#define LED_BLUE             PIN_LED1 // bluetooth
#define LED_GREEN            PIN_LED1 

#define LED_STATE_ON         1 // State when LED is litted

// Notice: blue LED (charge indicator) is not accessible

#define HAS_BUTTON 0

/*
 * Buttons
 */
// no Buttons
//#define BUTTON_1          (18)  // unusable: RESET
//#define BUTTON_2          (19)  // no connection
//#define BUTTON_3

/*
#define PIN_BUTTON1 0xFF // Pin for button on E-ink button module or IO expansion
#define BUTTON_NEED_PULLUP
#define PIN_BUTTON2 0xFF
#define PIN_BUTTON3 0xFF
#define PIN_BUTTON4 0xFF
*/

/*
 * Analog pins
 */
#define PIN_A0             (2)
#define PIN_A1             (3)
#define PIN_A2             (4)
#define PIN_A3             (5)
#define PIN_A4             (28)
#define PIN_A5             (29)
#define PIN_A6             (30)
#define PIN_A7             (31)


//#define PIN_VBAT             (4)  // PIN_A2

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
#define ADC_RESOLUTION    14

/*
 * Serial interfaces
 */

#define PIN_SERIAL1_RX      (8) // P0.08
#define PIN_SERIAL1_TX      (6) // P0.06

#define PIN_SERIAL2_RX      (33) // P1.01 (TOP Header)
#define PIN_SERIAL2_TX      (34) // P1.02 (TOP Header)


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI1_MISO       (43)  // P1.11
#define PIN_SPI1_MOSI       (35)  // P1.13
#define PIN_SPI1_SCK        (47)  // P1.15

#define PIN_SPI_MISO        (11)  // P0.11
#define PIN_SPI_MOSI        (32)  // P1.00
#define PIN_SPI_SCK         (24)  // P0.24


static const uint8_t SS   = (22);  // P0.22
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t SCK  = PIN_SPI_SCK ;

// RFM95/SX127x
#define USE_RF95 

#define LORA_DIO0           PIN_A5  // NC   for SX1262/SX1268
#define LORA_DIO1           PIN_A7  // IRQ  for SX1262/SX1268
#define LORA_DIO2                   // BUSY for SX1262/SX1268
#define LORA_DIO3                   // 
#define LORA_RESET          (36)    // P1.04

#define LORA_SCK            SCK
#define LORA_MISO           MISO
#define LORA_MOSI           MOSI
#define LORA_CS             SS

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (17)  // 0.17
#define PIN_WIRE_SCL         (20)  // 0.20

//#define PIN_WIRE1_SDA        (-1)
//#define PIN_WIRE1_SCL        (-1)

// enables 3.3V periphery like GPS or IO Module
//#define PIN_3V3_EN (13)


#define BATTERY_PIN PIN_A0
// and has 12 bit resolution
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0
// Definition of milliVolt per LSB => 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_MV_PER_LSB (0.73242188F)
// Voltage divider value => 1.330M + 1M voltage divider on VBAT = (1.5M / (1M + 1.5M))
#define VBAT_DIVIDER (0.399399399)
// Compensation factor for the VBAT divider
#define VBAT_DIVIDER_COMP (1.73F)
// Fixed calculation of milliVolt from compensation value
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.0
#define VBAT_AR_INTERNAL AR_INTERNAL_3_0
#define ADC_MULTIPLIER VBAT_DIVIDER_COMP // REAL_VBAT_MV_PER_LSB
#define VBAT_RAW_TO_SCALED(x) (REAL_VBAT_MV_PER_LSB * x)


/*
 * Meshtastic Architecture
 */
#define PRIVATE_HW

#define HAS_BLUETOOTH     1
#define HAS_SCREEN        0
#define HAS_WIRE          1
#define HAS_GPS           0
#define HAS_SENSOR        0
#define HAS_RADIO         1
#define HAS_CPU_SHUTDOWN  1
#define HAS_TELEMETRY     1 // Mandatory, will not compile if missing or 0 (Version 2.3.2)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif // End of _VARIANT_NICE_NANO_