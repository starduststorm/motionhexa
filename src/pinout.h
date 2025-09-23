#ifndef PINOUT_H
#define PINOUT_H

#define SDA 8
#define SCL 9

int photosensorNearbyPixels[] = {
  10,9,32, // adjacent
  8,33,11,31,34, // next arc
  7,12,30,35, /**/ 57,58,59, // only relevant up to px 35 at brightness 0x15
  60,61,62,63,64,56,36,29,13,6, // higher than 0x15
  // 90,89,88,87,86,85,65,54,38,27,15,4, // next arc only relevant at even higher brightness
  // beyond this there is little to no impact even at 0xFF brightness with a front case on
};

#if HARDWARE_VERSION == 5

#define PDM_BCLK 20
#define PDM_LRCLK (BCLK+1)
#define PDM_DATA 23

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 18

#define PHOTOSENSOR_POWER_PIN 25
#define PHOTOSENSOR_READ_PIN 27

#define BUTTON_0 24
#define BUTTON_PRESSED_STATE HIGH

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define VBUS_SENSOR_PIN 5
#define BATTERY_VOLTAGE_PIN 29

#define CHRG_PIN 16
#define GPOUT_PIN 1

#define EN_LDO_PIN 22
#define EN_BOOST_PIN 0
#define DISABLE_CHARGE_PIN 17


#elif HARDWARE_VERSION == 4

#define PDM_BCLK 23
#define PDM_LRCLK (BCLK+1)
#define PDM_DATA 25

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 18

#define PHOTOSENSOR_POWER_PIN 20
#define PHOTOSENSOR_READ_PIN 27

#define BUTTON_0 21
#define BUTTON_PRESSED_STATE HIGH

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define PWR_SWITCH_PIN 28
#define VBUS_SENSOR_PIN 5
#define BATTERY_VOLTAGE_PIN 29

#define CHRG_PIN 16
#define GPOUT_PIN 1

#define EN_LDO_PIN 22
#define EN_BOOST_PIN 0
#define DISABLE_CHARGE_PIN 17

#elif HARDWARE_VERSION == 3

#define PDM_BCLK 23
#define PDM_LRCLK (BCLK+1)
#define PDM_DATA 25

#define UNCONNECTED_PIN_1 29

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 22

#define PHOTOSENSOR_POWER_PIN 26
#define PHOTOSENSOR_READ_PIN 27

#define BUTTON_0 17
#define BUTTON_PRESSED_STATE LOW

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define PWR_SWITCH_PIN 28
#define VBUS_SENSOR_PIN 5

#define CHRG_PIN 16
#define GPOUT_PIN 0

#elif HARDWARE_VERSION == 2 // first form-factor rev

#define I2S_BCLK 23
#define I2S_LRCLK (BCLK+1)
#define I2S_DATA 25

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 22

#define PHOTOSENSOR_POWER_PIN 27
#define PHOTOSENSOR_READ_PIN 28

#define BATTERY_VOLTAGE_PIN 29

#define BUTTON_0 16
#define BUTTON_PRESSED_STATE LOW

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#else // first hardware rev

#define I2S_BCLK 1
#define I2S_LRCLK (BCLK+1)
#define I2S_DATA 3

#define UNCONNECTED_PIN_1 27

// These SPI pins are swapped from spec - FML but does not prevent FastLED from working
#define LED_SPI0_SCK 19
#define LED_SPI0_TX 18

#define PHOTOSENSOR_POWER_PIN 28
#define PHOTOSENSOR_READ_PIN 29

#define BUTTON_0 25
#define BUTTON_PRESSED_STATE LOW

#endif // pinout.

#endif
