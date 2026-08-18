#ifndef PINOUT_H
#define PINOUT_H

#define SDA 8
#define SCL 9

#if HARDWARE_VERSION >= 7
// v7 has three phototransistors, one per alternating hexagon corner. Nearby pixels are listed per sensor, nearest first, out to
// ~15mm (the v6 list below only mattered out to ~4 pixel spacings). These are geometric; the per-pixel effect on each sensor
// still needs to be measured with the form-factor diffuser (see PhotoSensorBrightness::measureBaseline).
// Sensor 0 / GPIO26 / Q3: logical top-left corner, next to px 0 and 10.
int photosensor0NearbyPixels[] = {
  10,0, // adjacent
  11,21,22,1, // next arc
  23,34,12,33,35,2, // ~9-11mm
  24,47,36,48,13,46,49,3, // ~12-15mm
};
// Sensor 1 / GPIO27 / Q2: logical bottom-left corner, next to px 250 and 261.
int photosensor1NearbyPixels[] = {
  250,261, // adjacent
  251,238,239,262, // next arc
  240,252,226,225,227,263, // ~9-11mm
  241,212,228,213,253,211,214,264, // ~12-15mm
};
// Sensor 2 / GPIO28 / Q1: logical right corner, next to px 144 (end of the center row) and 125.
int photosensor2NearbyPixels[] = {
  144,125, // adjacent
  143,162,107,124, // next arc
  161,142,106,179,90,123, // ~9-11mm
  160,178,141,89,105,195,74,122, // ~12-15mm
};
int *photosensorNearbyPixelLists[] = { photosensor0NearbyPixels, photosensor1NearbyPixels, photosensor2NearbyPixels };
int photosensorNearbyPixelCounts[] = {
  sizeof(photosensor0NearbyPixels)/sizeof(int),
  sizeof(photosensor1NearbyPixels)/sizeof(int),
  sizeof(photosensor2NearbyPixels)/sizeof(int),
};
#else
// v1-6: single phototransistor near px 9/10 (logical top-right corner)
int photosensorNearbyPixels[] = {
  10,9,32, // adjacent
  8,33,11,31,34, // next arc
  7,12,30,35, /**/ 57,58,59, // only relevant up to px 35 at brightness 0x15
  60,61,62,63,64,56,36,29,13,6, // higher than 0x15
  // 90,89,88,87,86,85,65,54,38,27,15,4, // next arc only relevant at even higher brightness
  // beyond this there is little to no impact even at 0xFF brightness with a front case on
};
int *photosensorNearbyPixelLists[] = { photosensorNearbyPixels };
int photosensorNearbyPixelCounts[] = { sizeof(photosensorNearbyPixels)/sizeof(int) };
#endif

#if HARDWARE_VERSION >= 7

#define EN_CHARGE 4 // pulled up, drive low to disable

#define PDM_CLK 20
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 22

#define UNCONNECTED_PIN_1 29 // FIXME: this adc pin is exported, but used as random seed. need another random seed strategy.

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 18

#define PHOTOSENSOR_POWER_PIN 15
#define PHOTOSENSOR_READ_PIN 26  // Q3, sensor 0
#define PHOTOSENSOR1_READ_PIN 27 // Q2, sensor 1
#define PHOTOSENSOR2_READ_PIN 28 // Q1, sensor 2
#define PHOTOSENSOR_COUNT 3

// BMI270 on spi0
#define IMU_MISO_PIN 0
#define IMU_CS_PIN 1
#define IMU_SCK_PIN 2
#define IMU_MOSI_PIN 3
#define IMU_SPI_HZ 8000000 // BMI270 max is 10MHz
// MMC5603NJ is on the 3.3V i2c bus (SDA/SCL above) at 0x30

#define BUTTON_0 24
#define BUTTON_PRESSED_STATE HIGH

#define MOTION_INT_PIN 5
#define LED_LINE_0_PWR_PIN 11

#define VBUS_SENSOR_PIN 10

#define GPOUT_PIN 12

#define EN_LDO_PIN 23

#elif HARDWARE_VERSION >= 5

#define PDM_CLK 20
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 23

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 18

#define PHOTOSENSOR_POWER_PIN 25
#define PHOTOSENSOR_READ_PIN 27
#define PHOTOSENSOR_COUNT 1

#define BUTTON_0 24
#define BUTTON_PRESSED_STATE HIGH

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define VBUS_SENSOR_PIN 5
#define BATTERY_VOLTAGE_PIN 29

#define CHRG_PIN 16
#define GPOUT_PIN 1

#define EN_LDO_PIN 22

#if HARDWARE_VERSION == 5
#define EN_BOOST_PIN 0
#endif
#define V6_DETECTOR_PIN 0

#define DISABLE_CHARGE_PIN 17


#elif HARDWARE_VERSION == 4

#define PDM_CLK 23
#define PDM_LRCLK (PDM_CLK+1)
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

#define PDM_CLK 23
#define PDM_LRCLK (PDM_CLK+1)
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

#define PDM_CLK 23
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 25

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
#define I2S_LRCLK (I2S_BCLK+1)
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

#ifndef PHOTOSENSOR_COUNT
#define PHOTOSENSOR_COUNT 1
#endif

#endif
