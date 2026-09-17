#pragma once
#ifndef COMPASSSTORE_H
#define COMPASSSTORE_H

// Persistent storage for the magnetometer hard-iron calibration (see compasscal.h / MotionManager.h).
//
// The board's hard iron (~44uT measured on v5/v6) exceeds the horizontal geomagnetic field (~23uT), so a heading is
// impossible until the offset is known. A converged calibration is persisted here and restored by MotionManager at init:
// on v7 as a software offset subtracted from the MMC5603NJ reading, on hardware < v7 as the ICM-20948 DMP compass bias registers,
// which start at zero every boot.
//
// EEPROM writes must happen from core0. core1 runs via arduino-pico loop1() to make core0 writes safe.

#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <stddef.h>

extern "C" uint8_t _EEPROM_start[]; // base of the EEPROM library's reserved 4KB flash sector (array: unsized, so gcc won't flag the read)

struct CompassBiasRecord {
  uint32_t magic;            // kCompassStoreMagic
  uint32_t version;          // kCompassStoreVersion
  uint32_t hardwareVersion;  // HARDWARE_VERSION the record was made with; the bias is in that revision's mag package frame
  int32_t biasCentiUT[3];    // hard-iron offset, magnetometer package frame, uT*100. calibrated = raw - bias
  uint32_t calCount;         // lifetime number of successful calibrations
  int32_t magSpreadCentiUT;  // |B| spread (max-min) at convergence, uT*100 (quality metric; lower is better)
  int32_t fitRadiusCentiUT;  // fitted |B| of the last accepted window, uT*100; the calibrator rejects fits that disagree with it
  uint32_t checksum;         // sum of all preceding 32-bit words
};

static constexpr uint32_t kCompassStoreMagic = 0x43504232;   // 'CPB2'
static constexpr uint32_t kCompassStoreVersion = 4;          // v1: ICM DMP register units; v2: no fit radius; v3: offset not at the thermal reference temperature. Not migrated

static inline uint32_t compassStoreChecksum(const CompassBiasRecord &rec) {
  const uint32_t *w = reinterpret_cast<const uint32_t *>(&rec);
  uint32_t sum = 0;
  for (size_t i = 0; i < (offsetof(CompassBiasRecord, checksum) / sizeof(uint32_t)); ++i) {
    sum += w[i];
  }
  return sum;
}

// Read + validate the persisted record. Safe on either core (pure XIP read). Returns false if no valid record is stored
// or it was made for a different hardware revision.
static inline bool compassStoreRead(CompassBiasRecord *out) {
  CompassBiasRecord rec;
  memcpy(&rec, _EEPROM_start, sizeof(rec));
  if (rec.magic != kCompassStoreMagic) return false;
  if (rec.version != kCompassStoreVersion) return false;
  if (rec.checksum != compassStoreChecksum(rec)) return false;
  if (rec.hardwareVersion != HARDWARE_VERSION) return false;
  if (out) *out = rec;
  return true;
}

// Write the record to flash. CORE0 ONLY (EEPROM.commit parks core1 for ~50ms). Fills in magic/version/checksum.
static inline bool compassStoreWrite(CompassBiasRecord rec) {
  rec.magic = kCompassStoreMagic;
  rec.version = kCompassStoreVersion;
  rec.hardwareVersion = HARDWARE_VERSION;
  rec.checksum = compassStoreChecksum(rec);
  EEPROM.begin(sizeof(CompassBiasRecord));
  EEPROM.put(0, rec);
  return EEPROM.commit();
}

// Invalidate any stored record. CORE0 ONLY. Takes effect on next boot (the running biases are left alone).
static inline bool compassStoreClear() {
  CompassBiasRecord rec;
  memset(&rec, 0, sizeof(rec));
  EEPROM.begin(sizeof(CompassBiasRecord));
  EEPROM.put(0, rec);
  return EEPROM.commit();
}

#endif // COMPASSSTORE_H
