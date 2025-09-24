#ifndef POWER_H
#define POWER_H

#include <util.h>
#include <patterning.h>

#include "pinout.h"

const int kFullCharge = 95; // %, limit for charging ui

#if HARDWARE_VERSION >= 3
#include <BQ27427.h>
#endif
struct BatteryData {
  uint16_t stateOfCharge;   // %
  uint16_t stateOfHealth;   // %
  uint16_t voltage;         // mV
  uint16_t currentCapacity; // mAh
  uint16_t fullCapacity;    // mAh
  int16_t powerDraw;        // mW
  uint16_t temperature;     // K
  uint16_t flags;
  void print() {
    logf("battery: %s, soc: %u%%, soh: %u%%, voltage: %umV, capacity: %umAh / %umAh, power: %imW, temp: %uK, flags: %X",
      batteryDetected()?"yes":"no", stateOfCharge, stateOfHealth, voltage, currentCapacity, fullCapacity, powerDraw, temperature, flags);
  }
  bool batteryDetected() {
    // Issue: almost always detects a battery, presumably mistaking the lipo charger, powering the load, as a battery
    //   if the charger is off then the i2c reads will be all 1s, SoC will be 0xFFFF.
    //   but there is no battery we might still see a reasoanble "state of charge" because the lipo charger is tricking the BQ27*.
    return stateOfCharge <= 100 && flags & 1<<3;
  }
};
const unsigned int BATTERY_CAPACITY = 2000;
BatteryData batteryData = {0};

class PowerManager {
  bool runningState = 0;
  unsigned long lastRunningChange=0;  // ms
  bool chargingState = 0;
  unsigned long lastChargingChange=0; // ms
  int knownChargePercent = 0;
  unsigned long lastFullChargeChange=0; // ms

  void setCharging(bool charging) {
    if (chargingState != charging) {
      logdf("Charging State Change %i -> %i, (battery:%i) since lastReachedFullCharge = %i", 
            chargingState, charging, batteryData.batteryDetected(), millis() - lastReachedFullCharge());
      chargingState = charging;
      lastChargingChange = millis();
    }
  }
public:
  bool batteryInitialized = false;
  
  // "running" means logical on-state, drawing design patterns, rather than drawing charging ui or powered on but not drawing.
  bool isRunning() {
    return runningState;
  }
  void setRunning(bool running) {
    runningState = running;
    lastRunningChange = millis();
  }
  unsigned long lastRunStateChange() {
    return lastRunningChange;
  }
  bool isCharging() { // usb powered and has a battery attached, even if fully charged
    return chargingState;
  }
  unsigned long lastChargingStateChange() {
    return lastChargingChange;
  }
  unsigned long lastReachedFullCharge() { // 0 if not "fully charged" to kFullCharge
    return lastFullChargeChange;
  }
  void update(bool vbusPowered, BatteryData &batteryData) {
    setCharging(vbusPowered && batteryInitialized && batteryData.batteryDetected());

    if (batteryData.stateOfCharge >= kFullCharge && knownChargePercent < kFullCharge) {
      logdf("Reached Full Charge!");
      lastFullChargeChange = millis();
    } else if (batteryData.stateOfCharge < kFullCharge-3) {
      lastFullChargeChange = 0;
    }
    knownChargePercent = batteryData.stateOfCharge;
  }
};

bool initializeBattery() {
  bool success = false;
#if HARDWARE_VERSION > 2
  // BQ27427 => 0x427
  // BQ27421 => 0x421
  logdf("coulomb counter deviceType = %X", lipo.deviceType());

  success = lipo.enterConfig(true);
  success &= lipo.setCapacity(BATTERY_CAPACITY);
  delay(5); // Hack: I don't know why a delay is required here but exitConfig fails without this
  // success &= lipo.setChemID(CHEM_B); // failing here, but hexa v5 has the bq27421YZFR-G1A which defaults to 4.2v lipo chemistry so we're good
  success &= lipo.exitConfig(true);
  
#endif
  return success;
}

BatteryData getBatteryData()
{
  assert(1 == get_core_num(), "getBatteryData not on core1");
  BatteryData data={0};
#if HARDWARE_VERSION >= 3
  // TODO: fetch only the data items we'll actually use, for perf
  data.stateOfCharge = lipo.soc(FILTERED);
  data.stateOfHealth = lipo.soh(PERCENT);
  data.voltage = lipo.voltage();
  data.currentCapacity = lipo.capacity(REMAIN);
  data.fullCapacity = lipo.capacity(FULL);
  data.powerDraw = lipo.power();
  data.temperature = lipo.temperature(INTERNAL_TEMP)/10.;
  data.flags = lipo.flags();
#endif
  return data;
}

uint8_t chargingPatternCheck(PatternRunner &runner, PowerManager &runState) {
  const int kChargePatternOverlayDuration = 1500; // how long to show charge pattern while running main patterns
  const int kFadeTime = 300;
  const int kSitTimeAtFullCharge = 5000;
  const int kRecentStateChangeDelay = 200;

  // This is whether or not the lipo charger is actually pushing current into the battery. I don't actually think this is useful since it toggles on and off a lot near full charge.
  // static unsigned long lastLipoChargeIndicator = 0;
  // if (!digitalRead(CHRG_PIN)) {
  //   lastLipoChargeIndicator = millis();
  // }
  
  // logf("isCharging=%i, isHexaRunning=%i, hasPattern = %i, lastReachedFullCharge= %i, lastChargingStateChange= %i, millis=%i", 
  //   runState.isCharging(), runState.isRunning(), (bool)(runner.pattern), runState.lastReachedFullCharge(), runState.lastChargingStateChange(), millis());

  uint8_t chargeAlpha = 0;

  if (runState.isCharging()) {
    unsigned long lastReachedFullCharge = runState.lastReachedFullCharge();
    if (runner.pattern) {
      int runTime = runner.pattern->runTime();
      if (runState.isRunning() && runTime > kChargePatternOverlayDuration) {
        // fade down overlay while device on
        chargeAlpha = max(0L, 0xFF - 0xFF * (long)(millis() - max(runState.lastRunStateChange(), runState.lastChargingStateChange())) / kFadeTime);
        logdf("fade down overlay device on => %i", chargeAlpha);

      } else if (lastReachedFullCharge && millis() - lastReachedFullCharge > kSitTimeAtFullCharge && runTime > kChargePatternOverlayDuration) {
        // fade down charging ui when fully charged
        int fadeProgress = min(millis() - lastReachedFullCharge - kSitTimeAtFullCharge, runTime - kChargePatternOverlayDuration);
        chargeAlpha = constrain(0xFF - 0xFF * (long)(fadeProgress) / kFadeTime, 0, 0xFF);
        logdf("fade down fully charged => %i", chargeAlpha);

      } else if (runTime < kFadeTime) {
        // fade up overlay
        chargeAlpha = min(0xFFL, (long)0xFF * runTime / kFadeTime);
        logdf("fade up overlay => %i", chargeAlpha);
      } else {
        // run overlay
        chargeAlpha = 0xFF;
      }
    } else if (
              (!runState.isRunning() && lastReachedFullCharge == 0) // not running, not fully charged
            ||  millis() - runState.lastChargingStateChange() < kRecentStateChangeDelay  // just plugged in
            || (millis() - runState.lastRunStateChange() < kRecentStateChangeDelay && lastReachedFullCharge == 0) // just turned off while plugged in
              ) {
      // start overlay
      chargeAlpha = 0x1;
      logdf("start overlay => %i", chargeAlpha);
    }
  } else {
    if (runner.pattern) {
      // fadedown overlay due to not charging
      chargeAlpha = max(0L, 0xFF - 0xFF * (long)(millis() - runState.lastChargingStateChange()) / kFadeTime);
      logdf("fade down overlay not charging => %i", chargeAlpha);
    }
  }
  // if (chargeAlpha) {
  //   logf("  charge overlay alpha = %02X", chargeAlpha);
  // }
  return chargeAlpha;
}

#if HARDWARE_VERSION >= 4
void powerOff() {
  gpio_put(EN_LDO_PIN, false);
  // should power off here, but delay here rather than forever loop, since sometimes we don't.
  while (digitalRead(BUTTON_0) == BUTTON_PRESSED_STATE) {
    // handle the button being left pressed after power off
    delay(50);
  }
  // should lose power here, but don't infinite loop in case we don't
  delay(500);
}
#endif

#endif
