#ifndef BENCH_H
#define BENCH_H

static inline void benchLoop(char *serialLine) {
  if (serialLine) {
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
    } else if (strcmp(serialLine, "POWERON") == 0) {
      logf("POWERON requested");
      if (!powerState.isRunning()) {
        startupCompleted();
      }
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

  if (compassLogging) {
    static unsigned long lastCompassLog = 0;
    if (millis() - lastCompassLog >= 500) {
      lastCompassLog = millis();
      Compass::logDiagnostics(MotionManager::motionFrame);
    }
  }
}

#endif
