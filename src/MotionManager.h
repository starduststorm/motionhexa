#pragma once

#include <functional>
#include <map>
#include <vector>
#include <Wire.h>

#include "hexaphysics.h"
#include "pinout.h"
#include "compasscal.h"
#include "compassstore.h"

// Motion sensing hardware by revision:
//   v7+:  BMI270 6-axis accel/gyro on SPI0 (GPIO0-3) + MMC5603NJ 3-axis magnetometer on I2C. Orientation is fused in software.
//   v1-6: ICM-20948 9-axis on I2C, with the on-chip DMP providing the orientation quaternion.
// MotionFrame is backend agnostic. localizeMotionFrame corrects for IMU placement.
// The magnetometer is hard-iron corrected on both (compasscal.h/compassstore.h); heading lives in compass.h.
#define MOTION_HW_BMI270_MMC5603 (HARDWARE_VERSION >= 7)
#define MOTION_HW_ICM20948 (HARDWARE_VERSION >= 1 && HARDWARE_VERSION < 7)

// Compiled-in hard-iron seed, uT, magnetometer package frame; all zero = none. Written and trusted at boot on a unit with
// no stored calibration so the compass arrow is available before the first tumble; the always-on calibrator
// (compasscal.h) corrects it from there. 
// FIXME: measure several units and fill this, but only if unit-to-unit spread is small (< ~20uT).
#if HARDWARE_VERSION >= 7
constexpr float kHexaDefaultMagBiasUT[3] = {0, 0, 0};
#else
constexpr float kHexaDefaultMagBiasUT[3] = {0, 0, 0};
#endif

#if MOTION_HW_BMI270_MMC5603
#include <SPI.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_MMC56x3.h>
#elif MOTION_HW_ICM20948
#include <ICM_20948.h>
#endif

struct Euler {
  float pitch = 0, roll = 0, yaw = 0; // degrees
};

struct Quaternion {
  float w = 1, x = 0, y = 0, z = 0;
};

// Chip-independent motion sample.
// Vector units are the ICM-20948 DMP-era LSB scales that the patterns were tuned against, regardless of the chip:
//   acc: 1/MotionManager::accelToGScale g per LSB (8192 LSB/g, i.e. ±4g full scale in int16)
//   gyr: 1/MotionManager::gyrToRadScale rad/s per LSB (16.4 LSB/dps, i.e. ±2000dps full scale)
//   mag: 1/MotionManager::magToUTScale µT per LSB (10 LSB/µT)
// Axes are the hexa's logical motion frame once localizeMotionFrame() has run, which MotionManager::loop() does
// internally before returning. quat/euler describe device orientation relative to a gravity-up world frame in those same axes.
struct MotionFrame {
  vector16 acc;
  // The same accelerometer reading in g, float, out to the sensor's own full scale (BMI270: ±16g) where acc saturates at ±4g.
  // For consumers that integrate impulses (a flick peaks well past 4g, and a clipped peak leaves a phantom net velocity).
  vectorf accG;
  vector16 gyr;
  vector16 mag;
  Euler euler;
  Quaternion quat;
  bool hasAccelGyro = false; // acc/gyr came from a live sensor this frame
  bool hasMag = false;       // mag came from a live sensor this frame
  bool hasOrientation = false; // quat/euler are valid
  float tempC = 0;           // IMU die temperature, a board temperature proxy for correlating magnetometer drift
  uint32_t magCount = 0;     // increments per fresh magnetometer sample; compare across frames to detect new data
  uint16_t magInitRetries = 0; // magnetometer bring-up retries since boot (v7; nonzero means it failed at boot)
  bool magCalibrated = false; // the hard-iron offset in effect is trusted (restored from flash or converged); heading usability is compass.h's call
  CompassCalStatus compassCal; // always-on hard-iron calibrator snapshot (core1 -> core0)
};

// The hexa's logical motion frame, expressed against the pixel geometry (hexGrid positions: x increases along a row, y
// increases toward row 0, z out of the LED face): +x_motion is toward the start of a row (geometric -x), +y_motion is toward
// the last row (geometric -y), +z_motion is out of the LED face. i.e. it is the geometry frame rotated 180deg about z, which
// is what makes accel read directly as "the direction things fall" for the physics patterns. Right-handed.
//
// Sensor packages sit wherever board routing wanted them, at whatever angle and on either side, so each one needs a fixed
// rotation from its own package axes into that frame. MotionManager itself knows nothing about the board: the caller hands it
// a MotionSensorPlacement in init() and localizeMotionFrame() applies it.

enum class Axis : int8_t { X = 1, Y = 2, Z = 3 };
constexpr Axis operator-(Axis a) { return (Axis)(-(int8_t)a); }

// A rotation from one sensor package's axes into the hexa motion frame: v_motion = m * v_chip.
struct AxisRotation {
  float m[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

  // Signed axis permutation, naming which signed chip axis feeds each motion axis. axes(-Axis::Y, -Axis::X, -Axis::Z) means
  // x_motion = -y_chip, y_motion = -x_chip, z_motion = -z_chip.
  static constexpr AxisRotation axes(Axis x, Axis y, Axis z) {
    AxisRotation r = {{{0,0,0},{0,0,0},{0,0,0}}};
    const Axis wants[3] = {x, y, z};
    for (int i = 0; i < 3; ++i) {
      int8_t a = (int8_t)wants[i];
      r.m[i][(a < 0 ? -a : a) - 1] = (a < 0 ? -1.f : 1.f);
    }
    return r;
  }

  // This rotation, followed by `degrees` counterclockwise about the motion frame's z axis (counterclockwise as seen looking at
  // the LED face). For packages whose in-plane angle isn't a multiple of 90deg.
  AxisRotation rotatedAboutZ(float degrees) const {
    const float c = cosf(degrees * (float)M_PI / 180.f), s = sinf(degrees * (float)M_PI / 180.f);
    AxisRotation r;
    for (int j = 0; j < 3; ++j) {
      r.m[0][j] = c * m[0][j] - s * m[1][j];
      r.m[1][j] = s * m[0][j] + c * m[1][j];
      r.m[2][j] = m[2][j];
    }
    return r;
  }

  vector16 apply(const vector16 &v) const {
    const float x = v.x, y = v.y, z = v.z;
    return vector16(saturate16(m[0][0]*x + m[0][1]*y + m[0][2]*z),
                    saturate16(m[1][0]*x + m[1][1]*y + m[1][2]*z),
                    saturate16(m[2][0]*x + m[2][1]*y + m[2][2]*z));
  }

  vectorf apply(const vectorf &v) const {
    return vectorf(m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
                   m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
                   m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z);
  }

  // A rotation by theta about axis n, viewed from the rotated frame, is the same angle about the rotated axis: the vector part
  // rotates and w is left alone.
  Quaternion apply(const Quaternion &q) const {
    return {q.w,
            m[0][0]*q.x + m[0][1]*q.y + m[0][2]*q.z,
            m[1][0]*q.x + m[1][1]*q.y + m[1][2]*q.z,
            m[2][0]*q.x + m[2][1]*q.y + m[2][2]*q.z};
  }

private:
  static int16_t saturate16(float v) {
    long r = lroundf(v);
    return (int16_t)(r > 32767 ? 32767 : (r < -32768 ? -32768 : r));
  }
};

// Where a board puts its motion sensors. Board-specific; see the hexa's own values in ledgraph.h.
struct MotionSensorPlacement {
  // accel/gyro package axes -> motion frame. Also applied to a chip-supplied orientation quaternion, which is in those axes.
  AxisRotation accelGyro;
  // magnetometer package axes -> motion frame. Its own entry since on some revisions it is a separate part at its own angle.
  AxisRotation mag;
  // Accel/gyro position relative to the center of the pixel array, in micrometers. In pixel geometry axes rather than motion
  // axes, since what consumes it (accelerationAtPixelIndex) works in pixel positions.
  UMPoint position;
  // Temperature coefficient of the board's hard iron, uT per degC, motion frame (what the COMPASS log shows), applied as
  // bias(T) = bias + slope * (T - magTempRefC) with T the IMU die temperature. Something near the v7 magnetometer is
  // thermally magnetic: 5uT/degC, linear over 30-47degC. All zero = no compensation. The stored hard-iron
  // offset is defined at magTempRefC.
  float magTempSlopeUTperC[3] = {0, 0, 0};
  float magTempRefC = 40;
};

class MotionManager : public MagBiasPort {
protected:
  static MotionManager *_singleton;
  MotionManager() {}
public:
  MotionManager(MotionManager &other) = delete;
  void operator=(const MotionManager &) = delete;
  static MotionManager &manager();

  // for external access, currently using this from core0 and the rest of the class from core1
  static MotionFrame motionFrame;

  // Unit scales for MotionFrame vectors (see MotionFrame).
  // 939.7 LSB/(rad/s): the ICM-20948 GYRO_FS_SEL=3 (±2000dps) scale, 16.4 LSB/dps
  constexpr static float gyrToRadScale = 16.4 * 180.0 / M_PI;
  // 8192 LSB/g: the ICM-20948 ACCEL_FS_SEL=1 (±4g) scale
  constexpr static float accelToGScale = 8192.0;
  // 10 LSB/µT
  constexpr static float magToUTScale = 10.0;

  // Software fusion (v7+): feed the magnetometer into the orientation filter. 
  // FIXME: test on hardware and calibrate
  bool fuseMagnetometer = false;

private:
  MotionSensorPlacement placement; // identity until init(); see localizeMotionFrame
  bool hasIMU = false;
  bool hasMagnetometer = false;
  MotionFrame frame; // store frame between loops since our framerate can exceed the sensor odr
  CompassCalibrator calibrator; // always-on hard-iron calibration, core1; see compasscal.h

  // Restore the persisted hard-iron offset, if any, into the chip; the calibrator trusts it until a tumble says otherwise.
  // Logs integers only (core1).
  void restoreMagBias() {
    CompassBiasRecord rec;
    if (compassStoreRead(&rec)) {
      float b[3] = {rec.biasCentiUT[0] / 100.0f, rec.biasCentiUT[1] / 100.0f, rec.biasCentiUT[2] / 100.0f};
      bool ok = writeMagBias(b);
      if (ok) calibrator.noteRestoredBias(b, rec.fitRadiusCentiUT / 100.0f);
      logf("  mag bias restore [%ld %ld %ld] x0.01uT R=%ld calCount=%lu spread=%ld: %s",
           (long)rec.biasCentiUT[0], (long)rec.biasCentiUT[1], (long)rec.biasCentiUT[2], (long)rec.fitRadiusCentiUT,
           (unsigned long)rec.calCount, (long)rec.magSpreadCentiUT, ok ? "ok" : "FAILED");
    } else if (kHexaDefaultMagBiasUT[0] != 0 || kHexaDefaultMagBiasUT[1] != 0 || kHexaDefaultMagBiasUT[2] != 0) {
      bool ok = writeMagBias(kHexaDefaultMagBiasUT);
      if (ok) calibrator.noteRestoredBias(kHexaDefaultMagBiasUT, 0);
      logf("  mag: no stored bias; seeded the hardware v%i default (%s)", HARDWARE_VERSION, ok ? "ok" : "FAILED");
    } else {
      logf("  mag: no stored bias; calibrating in the background (tumble the device)");
    }
  }

#if MOTION_HW_ICM20948
  const bool enableDMP = true;
  ICM_20948_I2C icm;

  // Hard iron lives in the DMP compass bias registers (uT * 2^16, ICM body frame), which the DMP subtracts before its own
  // fusion and before the Compass_Calibr output. The registers start at zero every boot.
  static constexpr float kCPassBiasScale = 65536.0f;
  bool readMagBias(float b[3]) override {
    int32_t r[3];
    bool ok = (icm.getBiasCPassX(&r[0]) == ICM_20948_Stat_Ok);
    ok &= (icm.getBiasCPassY(&r[1]) == ICM_20948_Stat_Ok);
    ok &= (icm.getBiasCPassZ(&r[2]) == ICM_20948_Stat_Ok);
    for (int i = 0; i < 3; ++i) b[i] = r[i] / kCPassBiasScale;
    return ok;
  }
  bool writeMagBias(const float b[3]) override {
    bool ok = (icm.setBiasCPassX((int32_t)(b[0] * kCPassBiasScale)) == ICM_20948_Stat_Ok);
    ok &= (icm.setBiasCPassY((int32_t)(b[1] * kCPassBiasScale)) == ICM_20948_Stat_Ok);
    ok &= (icm.setBiasCPassZ((int32_t)(b[2] * kCPassBiasScale)) == ICM_20948_Stat_Ok);
    return ok;
  }
  void initDMP() {
    logf("Init DMP...");
    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

    // 32-bit DMP-calibrated compass (Compass_Calibr packets, uT * 2^16) is our only usable mag source while the DMP runs
    // (see readHardware). NOTE: do NOT enable INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, it
    // perturbs the DMP: compass runtime calibration never engaged (biases stuck at 0), header2 compass accuracy read
    // garbage, and the fusion slewed the attitude at up to ~26 deg/s while the device was physically stationary.
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // Value = (DMP running rate / ODR ) - 1
    success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    // DMP runs at 55Hz; 10 ≈ 5Hz. FIFO output rate only; fusion still consumes the mag at the 69Hz set by initializeDMP.
    success &= (icm.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 10) == ICM_20948_Stat_Ok);

    success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
    success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
    success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

    if (success) {
      logf("DMP enabled!");
    } else {
      logf("Enable DMP failed!");
    }

    // Bias register writes go after the reset sequence, per SparkFun Example11.
    restoreMagBias();
  }

  bool initHardware() {
    icm.begin(Wire, 0);
    hasIMU = (icm.status == ICM_20948_Stat_Ok);
    hasMagnetometer = hasIMU;
    logf("  ICM20948 init = %i", hasIMU);
    if (hasIMU && enableDMP) {
      initDMP();
    }
    return hasIMU;
  }

  void readHardware() {
    ICM_20948_AGMT_t agmt = icm.getAGMT();
    // ICM DMP config is ±4g / ±2000dps, i.e. exactly the MotionFrame scales, so raw LSBs pass through
    frame.acc = vector16(agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z);
    frame.accG = vectorf(agmt.acc.axes.x / accelToGScale, agmt.acc.axes.y / accelToGScale, agmt.acc.axes.z / accelToGScale);
    frame.gyr = vector16(agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z);
    frame.hasAccelGyro = true;
    frame.tempC = agmt.tmp.val / 333.87f + 21.0f; // datasheet: 333.87 LSB/degC, 21degC offset
    // NOTE: agmt.mag is garbage whenever the DMP is enabled: initializeDMP reconfigures I2C_SLV0 to read 10 byte-swapped
    // bytes from the AK09916's undocumented RSV2 register, but getAGMT still parses EXT_SLV_SENS_DATA assuming the non-DMP
    // little-endian layout. The mag comes from the DMP's Compass_Calibr FIFO output below instead.
    if (!enableDMP) {
      // AK09916 is 0.15µT/LSB -> 10 LSB/µT
      frame.mag = vector16(agmt.mag.axes.x * 3 / 2, agmt.mag.axes.y * 3 / 2, agmt.mag.axes.z * 3 / 2);
      frame.hasMag = true;
      frame.magCount++;
      return;
    }

    // Drain the FIFO so we always end on the freshest sample; reading a single packet per loop falls behind permanently
    // once the FIFO backs up during a slow pattern frame.
    icm_20948_DMP_data_t data;
    do {
      icm.readDMPdataFromFIFO(&data);
      if ((icm.status != ICM_20948_Stat_Ok) && (icm.status != ICM_20948_Stat_FIFOMoreDataAvail)) {
        break;
      }
      processDMPPacket(data);
    } while (icm.status == ICM_20948_Stat_FIFOMoreDataAvail);
  }

  void processDMPPacket(const icm_20948_DMP_data_t &data) {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // The quaternion data is scaled by 2^30.
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
      double q0sq = 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
      if (q0sq < 0.0) q0sq = 0.0;
      double q0 = sqrt(q0sq);
      frame.quat = {(float)q0, (float)q1, (float)q2, (float)q3};
      frame.hasOrientation = true;
    }
    // DMP-calibrated compass (uT * 2^16, ICM body frame, hard-iron biases already subtracted)
    if (data.header & DMP_header_bitmap_Compass_Calibr) {
      float m[3] = {
        data.Compass_Calibr.Data.X / kCPassBiasScale,
        data.Compass_Calibr.Data.Y / kCPassBiasScale,
        data.Compass_Calibr.Data.Z / kCPassBiasScale,
      };
      frame.mag = vector16(clamp16(m[0] * magToUTScale), clamp16(m[1] * magToUTScale), clamp16(m[2] * magToUTScale));
      frame.hasMag = true;
      frame.magCount++;
      calibrator.onMagSample(m);
    }
  }

  // called on a localized copy of frame
  void finishFrame(MotionFrame &out) {
    if (out.hasOrientation) {
      out.euler = eulerFromQuaternion(out.quat);
    }
  }

#elif MOTION_HW_BMI270_MMC5603

  BMI270 imu;
  Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
  unsigned long lastFusionMicros = 0;

  // Hard iron is a software offset, uT, MMC5603NJ package frame, subtracted from every reading, plus a temperature term
  // (placement.magTempSlopeUTperC rotated into the package frame at init; the map is orthogonal so its inverse is its transpose).
  float magBiasUT[3] = {0, 0, 0};
  float magTempSlopePkg[3] = {0, 0, 0};
  float lastRawMagUT[3] = {0, 0, 0}; // continuous mode has no data-ready flag we use; a changed reading is a fresh sample
  bool readMagBias(float b[3]) override {
    for (int i = 0; i < 3; ++i) b[i] = magBiasUT[i];
    return true;
  }
  bool writeMagBias(const float b[3]) override {
    for (int i = 0; i < 3; ++i) magBiasUT[i] = b[i];
    return true;
  }

public:
  // core1, bench diagnostic (MAGSET serial command): pulse the MMC5603NJ SET/RESET coils and resume continuous mode. A step
  // in the reading afterwards means the drift was the AMR bridge offset (fixable with periodic auto SET/RESET); no step
  // means it is external to the sensor.
  void magSetReset() {
    if (!hasMagnetometer) return;
    mag.magnetSetReset();      // writes CTRL0 directly, which drops Cmm_freq_en
    mag.setContinuousMode(true);
    logf("mag SET/RESET pulsed");
  }
private:

  bool initHardware() {
    // BMI270 on SPI0. GPIO0=RX(SDO), GPIO1=CS, GPIO2=SCK, GPIO3=TX(SDI). 
    // The SparkFun BMI270 constructor leaves its bmi2_dev struct uninitialized, and bmi270_init() only assigns
    // config_file_ptr when it is NULL. After a warm reset that memory happens to be zero (or last boot's pointer) and it
    // works; on a cold boot it is SRAM garbage, the 8KB config upload streams from a wild pointer and core1 hard-faults
    // (BusFault in writeRegistersSPI). Zero it explicitly. BMI270 has no vtable and a do-nothing ctor, so this is safe.
    memset((void *)&imu, 0, sizeof(imu));
    SPI.setRX(IMU_MISO_PIN);
    SPI.setCS(IMU_CS_PIN);
    SPI.setSCK(IMU_SCK_PIN);
    SPI.setTX(IMU_MOSI_PIN);
    SPI.begin();
    int8_t err = imu.beginSPI(IMU_CS_PIN, IMU_SPI_HZ, SPI);
    hasIMU = (err == BMI2_OK);
    logf("  BMI270 init = %i (err %i)", hasIMU, err);
    if (hasIMU) {
      // ±16g so MotionFrame::accG carries a whole flick; MotionFrame::acc still clamps at the ICM-era ±4g so clipping behavior
      // there matches. ±2000dps as on the ICM. ODR comfortably above our frame rate.
      bmi2_sens_config configs[2];
      configs[0].type = BMI2_ACCEL;
      configs[1].type = BMI2_GYRO;
      if (imu.getConfigs(configs, 2) == BMI2_OK) {
        configs[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
        configs[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
        configs[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
        configs[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
        configs[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
        configs[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
        configs[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
        configs[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
        configs[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
        err = imu.setConfigs(configs, 2);
        logf("  BMI270 config = %i", err);
      }
      imu.disableAdvancedPowerSave();
    }

    initMagnetometer();
    logf("  MMC5603NJ init = %i", hasMagnetometer);
    return hasIMU || hasMagnetometer;
  }

  // Bring up the MMC5603NJ. Retry with logging to find intermittent init issues.
  unsigned long lastMagRetryMillis = 0;
  uint16_t magRetries = 0;
  void initMagnetometer() {
    hasMagnetometer = mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire);
    if (hasMagnetometer) {
      // continuous mode so reads never block waiting on a one-shot conversion; 200Hz keeps up with the frame loop.
      // begin() already pulsed SET/RESET once to clear the sensor's own bridge offset; the board's hard iron is ours.
      mag.setDataRate(200);
      mag.setContinuousMode(true);
      restoreMagBias();
    }
  }
  // Raw probe for the retry log: does 0x30 ACK, and what product ID does it report (0x10 expected)?
  void probeMagnetometer(int *ack, int *productId) {
    Wire.beginTransmission(MMC56X3_DEFAULT_ADDRESS);
    Wire.write(0x39); // MMC56X3_PRODUCT_ID
    *ack = Wire.endTransmission(false);
    *productId = -1;
    if (Wire.requestFrom((uint8_t)MMC56X3_DEFAULT_ADDRESS, (uint8_t)1) == 1) *productId = Wire.read();
    else Wire.endTransmission(true);
  }
  void retryMagnetometer() {
    if (hasMagnetometer || millis() - lastMagRetryMillis < 3000) return;
    lastMagRetryMillis = millis();
    int ack, id;
    probeMagnetometer(&ack, &id);
    initMagnetometer();
    magRetries++;
    logf("MMC5603NJ retry #%u: probe ack=%i (0=ok) productId=0x%02x, begin=%i", magRetries, ack, id, hasMagnetometer);
  }

public:
  // core1, bench diagnostic (I2CSCAN serial command): list the addresses that ACK on the Wire bus.
  void i2cScan() {
    char line[160]; int n = 0;
    n += snprintf(line + n, sizeof(line) - n, "I2C scan:");
    for (uint8_t a = 1; a < 127 && n < (int)sizeof(line) - 6; ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission(true) == 0) n += snprintf(line + n, sizeof(line) - n, " 0x%02x", a);
    }
    logf("%s%s", line, n <= 10 ? " (nothing)" : "");
  }
private:

  void readHardware() {
    if (hasIMU) {
      if (imu.getSensorData() == BMI2_OK) {
        // SparkFun driver hands back g and dps; rescale to MotionFrame LSB units
        frame.acc = vector16(clamp16(imu.data.accelX * accelToGScale), clamp16(imu.data.accelY * accelToGScale), clamp16(imu.data.accelZ * accelToGScale));
        frame.accG = vectorf(imu.data.accelX, imu.data.accelY, imu.data.accelZ);
        constexpr float dpsToLSB = gyrToRadScale * M_PI / 180.0;
        frame.gyr = vector16(clamp16(imu.data.gyroX * dpsToLSB), clamp16(imu.data.gyroY * dpsToLSB), clamp16(imu.data.gyroZ * dpsToLSB));
        frame.hasAccelGyro = true;
      }
      static unsigned long lastTempMillis = 0;
      if (millis() - lastTempMillis >= 1000) { // two SPI register reads; once a second is plenty
        lastTempMillis = millis();
        float t;
        if (imu.getTemperature(&t) == BMI2_OK) frame.tempC = t;
      }
    }
    retryMagnetometer();
    if (hasMagnetometer) {
      sensors_event_t event;
      if (mag.getEvent(&event)) {
        float raw[3] = {event.magnetic.x, event.magnetic.y, event.magnetic.z};
        bool fresh = (raw[0] != lastRawMagUT[0] || raw[1] != lastRawMagUT[1] || raw[2] != lastRawMagUT[2]);
        bool compensated = frame.tempC != 0 || (magTempSlopePkg[0] == 0 && magTempSlopePkg[1] == 0 && magTempSlopePkg[2] == 0);
        if (fresh && compensated) { // an uncompensated sample (no die temperature yet, first frames after boot) is tens of uT off
          float m[3];
          float dT = frame.tempC - placement.magTempRefC;
          for (int i = 0; i < 3; ++i) {
            lastRawMagUT[i] = raw[i];
            m[i] = raw[i] - magBiasUT[i] - magTempSlopePkg[i] * dT;
          }
          frame.mag = vector16(clamp16(m[0] * magToUTScale), clamp16(m[1] * magToUTScale), clamp16(m[2] * magToUTScale));
          frame.hasMag = true;
          frame.magCount++;
          calibrator.onMagSample(m);
        }
      }
    }
  }

  // called on a localized copy of frame, so the fused orientation comes out in hexa axes
  void finishFrame(MotionFrame &out) {
    if (out.hasAccelGyro) {
      fuse(out);
    }
  }

  // Madgwick AHRS (x-io), running in the localized hexa frame. q maps hexa-frame vectors into a gravity-up world frame,
  // which is the same convention as the ICM DMP's rotation vector.
  float fusionBeta = 0.08f;
  unsigned long fusionStartMicros = 0;
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  static float invSqrt(float x) { return 1.0f / sqrtf(x); }
  void fuse(MotionFrame &f) {
    unsigned long now = micros();
    if (lastFusionMicros == 0) {
      lastFusionMicros = now;
      fusionStartMicros = now;
      return;
    }
    float dt = (now - lastFusionMicros) * 1e-6f;
    lastFusionMicros = now;
    if (dt <= 0 || dt > 0.5f) return; // stalled; don't integrate garbage
    // trust the accelerometer heavily for the first second so we snap to gravity instead of slewing from identity
    float beta = (now - fusionStartMicros < 1000000ul) ? 2.0f : fusionBeta;

    float gx = f.gyr.x / gyrToRadScale, gy = f.gyr.y / gyrToRadScale, gz = f.gyr.z / gyrToRadScale;
    float ax = f.acc.x, ay = f.acc.y, az = f.acc.z;
    float mx = f.mag.x, my = f.mag.y, mz = f.mag.z;
    bool useMag = fuseMagnetometer && f.hasMag && !(mx == 0.0f && my == 0.0f && mz == 0.0f);

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
      float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
      float s0, s1, s2, s3;
      float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
      float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
      if (useMag) {
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
        float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my, _2q0mz = 2.0f * q0 * mz, _2q1mx = 2.0f * q1 * mx;
        float _2q0q2 = 2.0f * q0 * q2, _2q2q3 = 2.0f * q2 * q3;
        float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3, q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;
        // Reference direction of Earth's magnetic field
        float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        float _2bx = sqrtf(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        float _4bx = 2.0f * _2bx, _4bz = 2.0f * _2bz;
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      } else {
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2, _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      }
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    f.quat = {q0, q1, q2, q3};
    f.euler = eulerFromQuaternion(f.quat);
    f.hasOrientation = true;
  }
#else
  // No motion hardware
  bool readMagBias(float b[3]) override { return false; }
  bool writeMagBias(const float b[3]) override { return false; }
  bool initHardware() {
    logf("  no motion hardware");
    return false;
  }
  void readHardware() { }
  void finishFrame(MotionFrame &out) { }
#endif

  static int16_t clamp16(float v) {
    return (int16_t)constrain(v, -32768.f, 32767.f);
  }

public:
  // core1. COMPASSCAL: discard the hard-iron offset in effect and recalibrate from zero (the calibrator is otherwise always
  // running; this is for the bench). Progress is published in MotionFrame::compassCal; core0 persists (compassCalLoop).
  void restartCompassCalibration() {
    if (!hasMagnetometer) {
      logf("compass cal: no magnetometer");
      return;
    }
    logf("compass cal: restarting from zero");
    calibrator.restart(*this);
  }

  static Euler eulerFromQuaternion(const Quaternion &q) {
    // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2
    double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    double q2sqr = q2 * q2;
    // roll (x-axis rotation)
    double t0 = +2.0 * (q0 * q1 + q2 * q3);
    double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
    double roll = atan2(t0, t1) * 180.0 / PI;
    // pitch (y-axis rotation)
    double t2 = +2.0 * (q0 * q2 - q3 * q1);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch = asin(t2) * 180.0 / PI;
    // yaw (z-axis rotation)
    double t3 = +2.0 * (q0 * q3 + q1 * q2);
    double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
    double yaw = atan2(t3, t4) * 180.0 / PI;
    Euler e;
    e.pitch = pitch;
    e.roll = roll;
    e.yaw = yaw;
    return e;
  }

  // core1 only. Wire (and on v7, SPI) must already be initialized.
  // `placement` is where this board's sensors sit; every frame returned by loop() is rotated by it.
  bool init(const MotionSensorPlacement &placement) {
    logf("motionManager INIT");
    this->placement = placement;
#if MOTION_HW_BMI270_MMC5603
    for (int i = 0; i < 3; ++i) {
      magTempSlopePkg[i] = 0;
      for (int k = 0; k < 3; ++k) magTempSlopePkg[i] += placement.mag.m[k][i] * placement.magTempSlopeUTperC[k];
    }
    if (placement.magTempSlopeUTperC[0] != 0 || placement.magTempSlopeUTperC[1] != 0 || placement.magTempSlopeUTperC[2] != 0) {
      logf("  mag thermal compensation [%.2f %.2f %.2f]uT/degC (package frame), offset defined at %.0fdegC",
           magTempSlopePkg[0], magTempSlopePkg[1], magTempSlopePkg[2], placement.magTempRefC);
    }
#endif
    bool ok = initHardware();
    return ok;
  }

  // Rotates a frame's vectors, and any chip-supplied orientation quaternion, from chip-native axes into the hexa's logical
  // motion frame. loop() calls this before any software fusion runs, so fused orientation comes out in hexa axes too.
  void localizeMotionFrame(MotionFrame &frame) const {
    frame.acc = placement.accelGyro.apply(frame.acc);
    frame.accG = placement.accelGyro.apply(frame.accG);
    frame.gyr = placement.accelGyro.apply(frame.gyr);
    frame.mag = placement.mag.apply(frame.mag);
    frame.quat = placement.accelGyro.apply(frame.quat);
  }

  bool hasSensor() { return hasIMU; }
  bool hasMagSensor() { return hasMagnetometer; }

  // core1 only. Reads the sensors and returns the latest frame localized to hexa axes.
  // `frame` itself is retained in chip axes between loops (sensor odr may be below our frame rate); the copy is what gets
  // localized so a retained DMP quaternion is not re-rotated every call.
  MotionFrame loop() {
    if (!hasIMU && !hasMagnetometer) {
      return MotionFrame();
    }
    frame.hasAccelGyro = false;
    frame.hasMag = false;
    readHardware();
    calibrator.loop(*this, hasMagnetometer);
#if MOTION_HW_BMI270_MMC5603
    frame.magInitRetries = magRetries;
#endif
    frame.compassCal = calibrator.status();
    frame.magCalibrated = calibrator.calibrated();
    MotionFrame out = frame;
    localizeMotionFrame(out);
    finishFrame(out);
    return out;
  }
};

MotionManager *MotionManager::_singleton = nullptr;
MotionFrame MotionManager::motionFrame;
MotionManager &MotionManager::manager() {
  if (_singleton==nullptr) {
    _singleton = new MotionManager();
  }
  return *_singleton;
}


// Splits the accelerometer reading into gravity and linear acceleration, both in g, motion frame. The gravity estimate is
// carried through rotation by the gyro and pulled back toward the accelerometer only while it reads about 1g and the device
// is barely rotating, so a flick, shove or spin lands in linear() instead of being mistaken for a tilt. It tracks the measured vector rather than a unit one, so
// accelerometer offset ends up in gravity and linear() settles to exactly zero at rest.
struct GravityTracker {
  vectorf gravity;
  bool seeded = false;
  static constexpr float kCorrectionTau = 0.5f; // s; how quickly a wrong estimate (gyro error after a violent move) bleeds off
  static constexpr float kTrustBandG = 0.08f;   // |acc| this far from 1g means it isn't showing us gravity, so ignore it
  static constexpr float kTrustGyroRad = 2.0f;  // rad/s; likewise while rotating this fast (the accelerometer sits off-center)

  void reset() { seeded = false; }

  void update(const MotionFrame &motion, float dtSeconds) {
    const vectorf &a = motion.accG;
    if (!seeded) {
      gravity = a;
      seeded = true;
      return;
    }
    // a world-fixed vector seen from the rotating body: dg/dt = -w x g. First order, so restore the norm afterwards.
    float wx = motion.gyr.x / MotionManager::gyrToRadScale, wy = motion.gyr.y / MotionManager::gyrToRadScale, wz = motion.gyr.z / MotionManager::gyrToRadScale;
    float gx = gravity.x, gy = gravity.y, gz = gravity.z;
    float normBefore = sqrtf(gx*gx + gy*gy + gz*gz);
    float rx = gx + (gy*wz - gz*wy) * dtSeconds;
    float ry = gy + (gz*wx - gx*wz) * dtSeconds;
    float rz = gz + (gx*wy - gy*wx) * dtSeconds;
    float normAfter = sqrtf(rx*rx + ry*ry + rz*rz);
    float renorm = (normAfter > 0.0001f ? normBefore / normAfter : 1.0f);
    rx *= renorm; ry *= renorm; rz *= renorm;

    float accMag = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
    float gyrMag = sqrtf(wx*wx + wy*wy + wz*wz);
    float trust = constrain(1.0f - fabsf(accMag - 1.0f) / kTrustBandG, 0.0f, 1.0f) * constrain(1.0f - gyrMag / kTrustGyroRad, 0.0f, 1.0f);
    float k = min(1.0f, trust * dtSeconds / kCorrectionTau);
    gravity = vectorf(rx + (a.x - rx) * k, ry + (a.y - ry) * k, rz + (a.z - rz) * k);
  }

  vectorf linear(const MotionFrame &motion) const {
    return vectorf(motion.accG.x - gravity.x, motion.accG.y - gravity.y, motion.accG.z - gravity.z);
  }
};
