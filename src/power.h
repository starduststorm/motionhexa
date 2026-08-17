#ifndef POWER_H
#define POWER_H

#include <util.h>
#include <patterning.h>

#include "pinout.h"

const int kFullCharge = 95; // %, limit for charging ui

// v5/v6 charger (LP28013HQVF-435) floats at 4.35V but the cells are 4.2V
#define SOFTWARE_CHARGE_LIMITER HARDWARE_VERSION >= 5

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
  uint8_t softFull;         // ChargeController declared the battery full (software 4.2V cutoff); always 0 without SOFTWARE_CHARGE_LIMITER
  uint8_t sampled;          // filled from a gauge read (vs. the zeroed initial value)
  uint16_t controlStatus;   // CONTROL_STATUS, for the log
  void print(uint16_t senseMV = 0) {
    logf("battery: %s, soc: %u%%, soh: %u%%, voltage: %umV, sense: %umV, capacity: %umAh / %umAh, power: %imW, temp: %uK, flags: %X, status: %X%s%s",
      batteryDetected()?"yes":"no", stateOfCharge, stateOfHealth, voltage, senseMV, currentCapacity, fullCapacity, powerDraw, temperature, flags,
      controlStatus, gaugingReady() ? "" : ", initializing", softFull ? ", soft-full" : "");
  }
  bool batteryDetected() {
    // Issue: almost always detects a battery, presumably mistaking the lipo charger, powering the load, as a battery
    //   if the charger is off then the i2c reads will be all 1s, SoC will be 0xFFFF.
    //   but there is no battery we might still see a reasoanble "state of charge" because the lipo charger is tricking the BQ27*.
    return stateOfCharge <= 100 && flags & 1<<3;
  }
  // For a few seconds after a gauge reset, soc reads 0 with capacity 0/0. SoC is RemainingCapacity
  // over FullChargeCapacity, so FCC == 0 is exactly "no valid percentage yet" (and it comes good
  // ~2s before CONTROL_STATUS INITCOMP does).
  bool gaugingReady() {
    return sampled && fullCapacity != 0;
  }
};
const unsigned int BATTERY_CAPACITY = 2000;
BatteryData batteryData = {0};

#if SOFTWARE_CHARGE_LIMITER
// BATTERY_SENSE (GPIO29/ADC3): BATSYS -20k- node -33k- GND
// Independent of the BQ27* gauge; useful to sanity-check its i2c voltage reading.
// Note: only call from core0 — the photosensor also muxes the (single) ADC from core0.
uint16_t batterySenseMV() {
  const uint32_t kSenseTopR = 20000, kSenseBottomR = 33000;
  uint32_t nodeMV = analogRead(BATTERY_VOLTAGE_PIN) * 3300ul / 1023;
  return nodeMV * (kSenseTopR + kSenseBottomR) / kSenseBottomR;
}
// core0 samples the sense pin here so core1's battery log can print it next to the gauge voltage
volatile uint16_t lastBatterySenseMV = 0;
#endif

class PowerManager {
  bool runningState = 0;
  unsigned long lastRunningChange=0;  // ms
  bool chargingState = 0;
  unsigned long lastChargingChange=0; // ms
  int knownChargePercent = 0;
  unsigned long lastFullChargeChange=0; // ms

  void setCharging(bool charging) {
    if (chargingState != charging) {
      logf("Charging State Change %i -> %i, (battery:%i) since lastReachedFullCharge = %i", 
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
    // The charging ui opens on the edge of lastChargingStateChange(), so don't declare charging
    // until the gauge is readable or the edge fires seconds before there's anything to draw.
    // Unplugged is unambiguous without the gauge.
    if (!vbusPowered) {
      setCharging(false);
    } else if (batteryData.gaugingReady()) {
      setCharging(batteryInitialized && batteryData.batteryDetected());
    }
    if (!batteryData.gaugingReady()) {
      return;
    }

    // ChargeController's software cutoff is authoritative for "full"
    int chargePercent = batteryData.softFull ? 100 : batteryData.stateOfCharge;
    if (chargePercent >= kFullCharge && knownChargePercent < kFullCharge) {
      logdf("Reached Full Charge!");
      lastFullChargeChange = millis();
    } else if (chargePercent < kFullCharge-3) {
      lastFullChargeChange = 0;
    }
    knownChargePercent = chargePercent;
  }
};
PowerManager powerState;

// Raw CHEM_ID readback values (not the same as the CHEM_A/B/C subcommand enum):
// BQ27427: 0x3230 = 4.35V (default), 0x1202 = 4.2V, 0x3142 = 4.4V
// BQ27421 chemistry is fixed per hardware variant instead (YZFR-G1A = 4.2V) and reads 0x0128.
const uint16_t kBQ27421DeviceType = 0x421;
const uint16_t kBQ27427DeviceType = 0x427;
const uint16_t kBQ27427ChemID4V2 = 0x1202;

// "BATRESET" serial command: full gauge reset + reconfigure, to shed learned state (FCC/Qmax).
// Parsed on core0; executed on core1, which owns i2c.
const char* kBatteryResetCommand = "BATRESET";
volatile bool batteryResetRequested = false;

// Gauge config versioning: the Design Capacity we program doubles as a config-generation marker.
// Versioning lasts while BQ2742* is powered, mismatch triggers a reset+reconfigure.
// Bump the generation whenever the config values in initializeBattery() change.
// Older firmware wrote a plain 2000 (at the wrong offset on the BQ27421); ROM default is 1340.
const uint16_t kGaugeConfigGeneration = 1;
uint16_t targetDesignCapacity() { return BATTERY_CAPACITY + kGaugeConfigGeneration; }

// Reads the marker without disturbing the gauge. Returns 0 on error.
// A normal-mode data memory block read (no config-update round trip, no soft reset), validated
// against a byte we never write with a known ROM default (State offset 5, Load Select/Mode = 0x81).
// The BQ27421 also has a DesignCapacity() command (0x3C) as a fallback; the BQ27427 doesn't.
const uint8_t kStateLoadSelectOffset = 5;
const uint8_t kStateLoadSelectDefault = 0x81;
uint16_t readGaugeDesignCapacity(uint16_t deviceType) {
  uint8_t loadSel = 0;
  bool peekOK = lipo.peekDataMemory(BQ27427_ID_STATE, kStateLoadSelectOffset, &loadSel, 1)
             && loadSel == kStateLoadSelectDefault;
  uint16_t peeked = peekOK ? lipo.peekDataMemoryWord(BQ27427_ID_STATE, lipo.designCapacityOffset()) : 0;
  uint16_t direct = 0;
  if (deviceType == kBQ27421DeviceType) {
    Wire.beginTransmission(BQ27427_I2C_ADDRESS);
    Wire.write(0x3C);
    if (Wire.endTransmission(true) == 0 && Wire.requestFrom((uint8_t)BQ27427_I2C_ADDRESS, (uint8_t)2) == 2) {
      uint16_t lsb = Wire.read();
      direct = (uint16_t)(Wire.read() << 8) | lsb;
    }
  }
  btlogf("[t=%lu] gauge marker: data memory %s (load select %02X) reads %u; 0x3C reads %u",
         millis(), peekOK ? "valid" : "INVALID", loadSel, peeked, direct);
  return peekOK ? peeked : direct;
}

// Configures the gauge on the first battery connection (ITPOR) or a config-generation change;
// otherwise a few reads confirm the marker and the running gauge is left alone. The config path
// blocks core1 for ~1.5s (mostly the gauge's soft reset on exitConfig), so should be a rare init path.
bool initializeBattery() {
  bool success = false;
#if HARDWARE_VERSION >= 5
  uint16_t deviceType = lipo.deviceType();
  logdf("coulomb counter deviceType = %X", deviceType);

  if (deviceType != kBQ27421DeviceType && deviceType != kBQ27427DeviceType) {
    // An absent gauge reads all 1s, which passes every flag test below and then spins
    // enterConfig/exitConfig for their full 2s timeouts each.
    logdf("no coulomb counter (deviceType = %X); skipping gauge config", deviceType);
    return false;
  }
  lipo.setDeviceType(deviceType); // BQ27421 (v5/v6) and BQ27427 (v7) lay out data memory differently

  bool itpor = lipo.itporFlag();
  btlogf("[t=%lu] initializeBattery: deviceType=%X itpor=%i", millis(), deviceType, itpor);
  if (!itpor) {
    uint16_t designCapacity = readGaugeDesignCapacity(deviceType);
    if (designCapacity == targetDesignCapacity()) {
      btlogf("[t=%lu] initializeBattery: fast path, gauge already at generation %u", millis(), kGaugeConfigGeneration);
      return true;
    }
    if (designCapacity == 0) {
      btlogf("[t=%lu] initializeBattery: design capacity read failed, deferring", millis());
      return true;
    }
    // Configured by older firmware: full reset to shed its learned state, then reconfigure
    // (reset sets ITPOR). Once per boot, so a failing config write can't reset-loop.
    static bool didMigrationReset = false;
    if (didMigrationReset) return true;
    didMigrationReset = true;
    logf("gauge config generation mismatch (design capacity %u, want %u): resetting gauge state",
         designCapacity, targetDesignCapacity());
    if (!lipo.reset()) return false;
    delay(100); // let the gauge finish re-initializing
  }

  unsigned long configStart = millis();
  btlogf("[t=%lu] initializeBattery: entering full config path", configStart);

  // BQ27427 defaults to the 4.35V chem profile; our cells are 4.2V.
  if (deviceType == kBQ27427DeviceType && (uint16_t)lipo.chemID() != kBQ27427ChemID4V2) {
    bool chemSuccess = lipo.setChemID(CHEM_B);
    btlogf("[t=%lu] initializeBattery: setChemID(CHEM_B) = %i", millis(), chemSuccess);
  }

  success = lipo.enterConfig(true);
  success &= lipo.setCapacity(targetDesignCapacity()); // BATTERY_CAPACITY + config-generation marker
  success &= lipo.setDesignEnergy(BATTERY_CAPACITY * 3.7f); // mWh, nominal 3.7V cell
  success &= lipo.setTerminateVoltage(3200); // mV; loaded voltage where SoC=0
  // Taper Rate = Design Capacity / (0.1 * taper current), taper current = 100mA:
  success &= lipo.setTaperRate(10 * BATTERY_CAPACITY / 100);
  delay(5); // Hack: I don't know why a delay is required here but exitConfig fails without this
  success &= lipo.exitConfig(true); // soft reset: gauge re-inits from a fresh OCV estimate
  btlogf("[t=%lu] initializeBattery: full config path done, success=%i, took %lums, marker reads %u (want %u)",
         millis(), success, millis() - configStart, readGaugeDesignCapacity(deviceType), targetDesignCapacity());
#endif
  return success;
}

#if SOFTWARE_CHARGE_LIMITER
/* Software 4.2V charge cutoff
 * v4-v6 has uses a LP28013HQVF-435 for charging, which charges to 4.35V, but our cells are 4.2V lipo.
 * termination IBF = RISET*IBAT/RIBF = 1.8k*1A/20k = 90mA, but this only applies after 4.35V float.
 * We'll use ~EN_CHARGE to disable the charger after reading 4.2V+, wait for cell relax, 
 * and mark it as soft-full when it remains near cutoff.
*/
/* Pacing: the pack has enough series resistance (~0.5ohm incl. cell ESR) that the moment the
 * charger enables, the measured terminal voltage jumps to the charger's 4.35V CV point even
 * with the cell nowhere near full — so the on-charge voltage is IR, not cell state, and is
 * useless as a cutoff signal. Instead, all decisions use the relaxed voltage (charger off,
 * settles in <5s), and charge runs in timed ON pulses whose duration shrinks
 * as the relaxed voltage approaches full. 
 */
const uint16_t kChargeFullMV    = 4150; // relaxed voltage at/above this => full
const uint16_t kRechargeMV      = 4050; // after full, start a new charge cycle below this
const unsigned long kChargeSettleMS = 5000; // relaxation time after an ON pulse before judging voltage

// ON-pulse duration from the last relaxed voltage: lenient in bulk, cautious near the top.
// Sized so a pulse can't skip past full: even 120s at 1A is ~33mAh (~1.7% SoC, a few tens
// of mV of OCV), and each pulse ends with a relaxed-voltage check.
unsigned long chargePulseMS(uint16_t relaxedMV) {
  if (relaxedMV < 4000) return 120000; // bulk: cell well below full
  if (relaxedMV < 4100) return 30000;
  return 10000;                        // approaching kChargeFullMV; charger current is tapered here anyway
}

// total pack series resistance (cell ESR + protection + leads + traces), from bench logs:
// ~500mV of relaxation at ~1A. used to un-sag loaded off-phase readings; bench-tune.
const uint16_t kPackResistanceMilliOhm = 500;

class ChargeController {
public:
  enum State : uint8_t { charging, settling, full };
private:
  State state = settling; // measure the relaxed voltage before the first pulse
  unsigned long stateEnteredAt = 0;
  uint16_t relaxedMV = 0; // last settled charger-off voltage (sags with system load; thresholds account for it)
  bool chargeEnabled = false; // setup() boots with the charger disabled

  void enterState(State newState) {
    state = newState;
    stateEnteredAt = millis();
  }
  void setChargeEnabled(bool enable, uint16_t mv) {
    if (enable != chargeEnabled) {
      chargeEnabled = enable;
      digitalWrite(DISABLE_CHARGE_PIN, !enable); // ~EN_CHARGE: high disables the charger
      logdf("charger %s (state=%u, %umV, relaxed %umV)", enable ? "enabled" : "disabled", state, mv, relaxedMV);
    }
  }
  static bool plausibleVoltage(uint16_t mv) {
    return mv > 2500 && mv < 4450; // filters 0 / 0xFFFF from failed i2c reads
  }
  // The charger is off in settling/full, so any measured power draw is system load sagging
  // the reading; estimate the unloaded OCV so pattern load while plugged can't hide a full
  // cell. Only ever raises the value => errs toward stopping charge early, the safe direction.
  static uint16_t estimatedRelaxedMV(BatteryData &bd) {
    uint32_t loadSagMV = (uint32_t)abs(bd.powerDraw) * kPackResistanceMilliOhm / bd.voltage;
    return bd.voltage + loadSagMV;
  }
public:
  bool isChargeEnabled() { return chargeEnabled; }
  bool isFull() { return state == full; }

  // Called from the core1 battery poll with fresh gauge data
  void update(BatteryData &bd) {
    if (!bd.batteryDetected() || !plausibleVoltage(bd.voltage)) {
      // No battery, or we can't read the gauge: don't charge what we can't supervise.
      setChargeEnabled(false, bd.voltage);
      enterState(settling); // re-measure before charging if readings return
      bd.softFull = false;
      return;
    }
    switch (state) {
      case charging:
        // timed pulse; on-charge voltage itself is ignored (it reads ~4.35V regardless)
        if (millis() - stateEnteredAt >= chargePulseMS(relaxedMV)) {
          enterState(settling);
        }
        break;
      case settling:
        if (millis() - stateEnteredAt < kChargeSettleMS) break; // let the cell relax first
        relaxedMV = estimatedRelaxedMV(bd);
        enterState(relaxedMV >= kChargeFullMV ? full : charging);
        break;
      case full:
        // No power path: plugged-in system load drains the battery until we top it back off.
        if (estimatedRelaxedMV(bd) < kRechargeMV) {
          relaxedMV = estimatedRelaxedMV(bd);
          enterState(charging);
        }
        break;
    }
    setChargeEnabled(state == charging, bd.voltage);
    bd.softFull = (state == full);
  }
};
ChargeController chargeController;
#endif // SOFTWARE_CHARGE_LIMITER

bool sampleBattery(BatteryData &out) {
  assert(1 == get_core_num(), "sampleBattery not on core1");
#if HARDWARE_VERSION >= 5
  // TODO: fetch only the data items we'll actually use, for perf
  BatteryData bd = {0};
  bd.stateOfCharge = lipo.soc(FILTERED);
  bd.stateOfHealth = lipo.soh(PERCENT);
  bd.voltage = lipo.voltage();
  bd.currentCapacity = lipo.capacity(REMAIN);
  bd.fullCapacity = lipo.capacity(FULL);
  bd.powerDraw = lipo.power();
  bd.temperature = lipo.temperature(INTERNAL_TEMP)/10.;
  bd.flags = lipo.flags();
  bd.controlStatus = lipo.status();
  bd.sampled = true;
  out = bd;
  return true;
#else
  return false;
#endif
}

#if HARDWARE_VERSION >= 5
#if DEBUG_BOOT_TIMING
// Times a blocking step on core1 and reports it if it ate more than a few motion frames.
#define CORE1_STEP(label, ...) do { \
    unsigned long _stepStart = millis(); \
    __VA_ARGS__; \
    unsigned long _stepMS = millis() - _stepStart; \
    if (_stepMS >= kStallLogMS) logf("[t=%lu] core1 stall: %s blocked %lums", millis(), label, _stepMS); \
  } while (0)
#else
#define CORE1_STEP(label, ...) do { __VA_ARGS__; } while (0)
#endif

// Motion is the QoS-critical stream on core1, so gauge i2c is one bounded step per motion frame,
// starting only once motion frames are flowing. Returns true when `bd` holds a fresh sample.
// `motionStartedAt` is when core1 began publishing motion frames.
const unsigned long kBatteryStartDelayMS = 100;
bool battery_step_core1(unsigned long motionStartedAt, BatteryData &bd) {
  assert(1 == get_core_num(), "battery_step_core1 not on core1");
  static bool batterySucess = false;
  static bool didInitialize = false;
  static unsigned long lastBatteryPoll = 0;

  if (millis() - motionStartedAt < kBatteryStartDelayMS) {
    return false;
  }
  if (!didInitialize) {
    // Normally a few reads; the gauge stays up on battery across power cycles, so it already
    // holds our config and a valid soc for the first sample.
    didInitialize = true;
    CORE1_STEP("initializeBattery", batterySucess = initializeBattery());
    powerState.batteryInitialized = batterySucess; // should be fine to write this bool from core1?
    logf("initializeBattery = %i", batterySucess);
    return false;
  }
  if (batteryResetRequested) {
    batteryResetRequested = false;
    // CONTROL_RESET reloads ROM defaults and sets ITPOR, so initializeBattery() takes its config path.
    bool resetOK = false;
    CORE1_STEP("BATRESET lipo.reset", resetOK = lipo.reset());
    delay(100); // let the gauge finish re-initializing before reconfiguring it
    CORE1_STEP("BATRESET initializeBattery", batterySucess = initializeBattery());
    powerState.batteryInitialized = batterySucess;
    logf("BATRESET: reset=%i reconfigure=%i", resetOK, batterySucess);
    lastBatteryPoll = 0; // poll fresh gauge data immediately
    return false;
  }
#if SOFTWARE_CHARGE_LIMITER
  // fast polls whenever supervising an active charge cycle: pulse timing and settle
  // sampling both want ~1s resolution. slow polls when unplugged or battery full.
  unsigned long batteryPollInterval = (digitalRead(VBUS_SENSOR_PIN) && !chargeController.isFull()) ? 1000 : 5000;
#else
  // charger terminates on its own; polls only feed soc/ui
  const unsigned long batteryPollInterval = 5000;
#endif
  if (lastBatteryPoll != 0 && millis() - lastBatteryPoll < batteryPollInterval) {
    return false;
  }
  if (!batterySucess) {
    // retry if we had a transient i2c failure at boot
    CORE1_STEP("initializeBattery retry", batterySucess = initializeBattery());
    powerState.batteryInitialized = batterySucess;
    if (batterySucess) {
      logf("initializeBattery retry succeeded");
      return false; // start sampling next frame rather than stacking i2c into this one
    }
    lastBatteryPoll = millis();
    // No readable gauge: publish an empty sample, which batteryDetected() reads as no battery.
    bd = {0};
    bd.sampled = true;
    return true;
  }
  bool sampled = false;
  CORE1_STEP("sampleBattery", sampled = sampleBattery(bd));
  lastBatteryPoll = millis();
  if (!sampled) {
    return false;
  }
#if SOFTWARE_CHARGE_LIMITER
  chargeController.update(bd);
  bd.print(lastBatterySenseMV);
#else
  bd.print();
#endif
  return true;
}
#endif // HARDWARE_VERSION >= 5


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

  // No gaugingReady() check here: the overlay is edge-triggered off lastChargingStateChange(), and
  // PowerManager::update() already withholds that edge until the gauge is readable.
  if (runState.isRunning() && runState.lastRunStateChange() > runState.lastChargingStateChange()) {
    // do not show the charging ui if we powered on right after plugging in
    return 0;
  }
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
