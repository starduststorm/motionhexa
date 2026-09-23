#ifndef BENCH_H
#define BENCH_H

#if AUTO_BRIGHTNESS
static bool autoBrightnessLogging = false;
#endif

static inline void benchLoop(char *serialLine) {
  if (serialLine) {
#if AUTO_BRIGHTNESS
    // photosensor / auto-brightness diagnostics:
    //   AB            toggle a diagnostics line every 500 ms (per-sensor counts, estimate, brightness, thermal cap)
    //   AB FAKE <n>   feed the controller a synthetic ambient of n ADC counts; AB FAKE -1 releases it
    //   AB THERM 0|1  run without / with the thermal ceiling
    if (strcmp(serialLine, "AB") == 0) {
      autoBrightnessLogging = !autoBrightnessLogging;
      logf("AB logging %s", autoBrightnessLogging ? "on" : "off");
    } else if (strncmp(serialLine, "AB FAKE ", 8) == 0) {
      float counts = atof(serialLine + 8);
      autoBrightness->injectedAmbient16 = (counts < 0 ? -1 : (int32_t)(counts * 16));
      logf("AB FAKE %.1f counts", counts);
    } else if (strncmp(serialLine, "AB THERM ", 9) == 0) {
      autoBrightness->thermalEnabled = atoi(serialLine + 9) != 0;
      logf("AB THERM %i", autoBrightness->thermalEnabled ? 1 : 0);
    } else
#endif
    if (strcmp(serialLine, "COMPASSCAL") == 0) {
      logf("COMPASSCAL: discarding the hard-iron offset in effect; tumble the device through every orientation");
      compassCalRequested = true;
    } else if (strcmp(serialLine, "COMPASSCLEAR") == 0) {
      logf("COMPASSCLEAR: %s (takes effect on next boot)", compassStoreClear() ? "ok" : "FAILED");
    } else if (strcmp(serialLine, "COMPASS") == 0) {
      compassLogging = !compassLogging;
      logf("COMPASS logging %s", compassLogging ? "on" : "off");
    }
#if MOTION_HW_BMI270_MMC5603
    else if (strcmp(serialLine, "MAGSET") == 0) {
      logf("MAGSET requested");
      magSetResetRequested = true;
    } else if (strcmp(serialLine, "I2CSCAN") == 0) {
      i2cScanRequested = true; // core1 owns Wire
    }
#endif
#if HARDWARE_VERSION >= 5
    else if (strcmp(serialLine, "POWEROFF") == 0) {
      logf("POWEROFF requested");
      Serial.flush();
      delay(50);
      powerOff();
#if HARDWARE_VERSION >= 7
    } else if (strncmp(serialLine, "GPOUTLOW ", 9) == 0) {
      // diagnostic: load the gauge's 1.8V regulator through R6 (GPOUT's 10k pull-up to gauge VDD) the way a resetting or
      // unpowered RP2350 does, without resetting anything. Watch FLAGS bit 5 (ITPOR, 0x20) in the battery log afterwards.
      int ms = constrain(atoi(serialLine + 9), 1, 30000);
      logf("GPOUTLOW: driving GPOUT low for %ims", ms);
      pinMode(GPOUT_PIN, OUTPUT);
      digitalWrite(GPOUT_PIN, LOW);
      unsigned long start = millis();
      while (millis() - start < (unsigned long)ms) { watchdog_update(); delay(10); }
      pinMode(GPOUT_PIN, INPUT);
      logf("GPOUTLOW: released");
#endif
    } else if (strcmp(serialLine, "REBOOT") == 0) {
      logf("REBOOT requested");
      Serial.flush();
      delay(50);
      watchdog_reboot(0, 0, 0);
    } else if (strcmp(serialLine, "POWERON") == 0) {
      logf("POWERON requested");
      if (!powerState.isRunning()) {
        startupCompleted();
      }
#if HARDWARE_VERSION >= 7
    } else if (strcmp(serialLine, "LOWBATT") == 0) {
      logf("LOWBATT: showing the refused power-on indication");
      if (!lowBatteryRunner) {
        refuseStartForLowBattery();
      }
#endif
    } else if (strcmp(serialLine, kBatteryResetCommand) == 0) {
      logf("BATRESET requested");
      batteryResetRequested = true;
    } else if (strncmp(serialLine, "PATTERN ", 8) == 0) {
      int patternIndex = atoi(serialLine + 8);
      logf("PATTERN %i requested", patternIndex);
      if (powerState.isRunning()) {
        indexedRunner->runPatternAtIndex(patternIndex);
      }
    } else if (strncmp(serialLine, "FAKEFFT ", 8) == 0) {
      fftProcessing.benchTestLevel = atoi(serialLine + 8);
      logf("FAKEFFT %i requested", fftProcessing.benchTestLevel);
    }
#endif
  }

#if AUTO_BRIGHTNESS
  if (autoBrightnessLogging) {
    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 500) {
      lastLog = millis();
      autoBrightness->logDiagnostics(batteryData.temperature);
    }
  }
#endif

  if (compassLogging) {
    static unsigned long lastCompassLog = 0;
    if (millis() - lastCompassLog >= 500) {
      lastCompassLog = millis();
      Compass::logDiagnostics(MotionManager::motionFrame);
    }
  }
}

#endif
