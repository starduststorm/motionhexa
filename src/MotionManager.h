#pragma once
#ifndef MOTIONMANAGER_H
#define MOTIONMANAGER_H

#include <functional>
#include <map>
#include <vector>
#include <Wire.h>

#include "hexaphysics.h"
#include "pinout.h"

// Motion sensing hardware by revision:
//   v7+:  BMI270 6-axis accel/gyro on SPI0 (GPIO0-3) + MMC5603NJ 3-axis magnetometer on I2C. Orientation is fused in software.
//   v1-6: ICM-20948 9-axis on I2C, with the on-chip DMP providing the orientation quaternion.
// MotionFrame is backend agnostic. localizeMotionFrame corrects for IMU placement.
#define MOTION_HW_BMI270_MMC5603 (HARDWARE_VERSION >= 7)
#define MOTION_HW_ICM20948 (HARDWARE_VERSION < 7)

#if MOTION_HW_BMI270_MMC5603
#include <SPI.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_MMC56x3.h>
#else
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
  vector16 gyr;
  vector16 mag;
  Euler euler;
  Quaternion quat;
  bool hasAccelGyro = false; // acc/gyr came from a live sensor this frame
  bool hasMag = false;       // mag came from a live sensor this frame
  bool hasOrientation = false; // quat/euler are valid
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
};

class MotionManager {
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

#if MOTION_HW_ICM20948
  const bool enableDMP = true;
  ICM_20948_I2C icm;
  void initDMP() {
    logf("Init DMP...");
    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);
    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // Value = (DMP running rate / ODR ) - 1
    success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
    success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
    success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

    if (success) {
      logf("DMP enabled!");
    } else {
      logf("Enable DMP failed!");
    }
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
    frame.gyr = vector16(agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z);
    // AK09916 is 0.15µT/LSB -> 10 LSB/µT
    frame.mag = vector16(agmt.mag.axes.x * 3 / 2, agmt.mag.axes.y * 3 / 2, agmt.mag.axes.z * 3 / 2);
    frame.hasAccelGyro = true;
    frame.hasMag = true;

    if (!enableDMP) {
      return;
    }
    icm_20948_DMP_data_t data;
    icm.readDMPdataFromFIFO(&data);

    if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail)) { // Was valid data available?
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
    }
  }

  // called on a localized copy of frame
  void finishFrame(MotionFrame &out) {
    if (out.hasOrientation) {
      out.euler = eulerFromQuaternion(out.quat);
    }
  }

#else // MOTION_HW_BMI270_MMC5603

  BMI270 imu;
  Adafruit_MMC5603 mag = Adafruit_MMC5603(12345);
  unsigned long lastFusionMicros = 0;

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
      // ±4g / ±2000dps: same full scale as the ICM config so clipping behavior matches. ODR comfortably above our frame rate.
      bmi2_sens_config configs[2];
      configs[0].type = BMI2_ACCEL;
      configs[1].type = BMI2_GYRO;
      if (imu.getConfigs(configs, 2) == BMI2_OK) {
        configs[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
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

    hasMagnetometer = mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire);
    logf("  MMC5603NJ init = %i", hasMagnetometer);
    if (hasMagnetometer) {
      // continuous mode so reads never block waiting on a one-shot conversion; 200Hz keeps up with the frame loop
      mag.setDataRate(200);
      mag.setContinuousMode(true);
    }
    return hasIMU || hasMagnetometer;
  }

  static int16_t clamp16(float v) {
    return (int16_t)constrain(v, -32768.f, 32767.f);
  }

  void readHardware() {
    if (hasIMU) {
      if (imu.getSensorData() == BMI2_OK) {
        // SparkFun driver hands back g and dps; rescale to MotionFrame LSB units
        frame.acc = vector16(clamp16(imu.data.accelX * accelToGScale), clamp16(imu.data.accelY * accelToGScale), clamp16(imu.data.accelZ * accelToGScale));
        constexpr float dpsToLSB = gyrToRadScale * M_PI / 180.0;
        frame.gyr = vector16(clamp16(imu.data.gyroX * dpsToLSB), clamp16(imu.data.gyroY * dpsToLSB), clamp16(imu.data.gyroZ * dpsToLSB));
        frame.hasAccelGyro = true;
      }
    }
    if (hasMagnetometer) {
      sensors_event_t event;
      if (mag.getEvent(&event)) {
        frame.mag = vector16(clamp16(event.magnetic.x * magToUTScale), clamp16(event.magnetic.y * magToUTScale), clamp16(event.magnetic.z * magToUTScale));
        frame.hasMag = true;
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
#endif

public:
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
    bool ok = initHardware();
    return ok;
  }

  // Rotates a frame's vectors, and any chip-supplied orientation quaternion, from chip-native axes into the hexa's logical
  // motion frame. loop() calls this before any software fusion runs, so fused orientation comes out in hexa axes too.
  void localizeMotionFrame(MotionFrame &frame) const {
    frame.acc = placement.accelGyro.apply(frame.acc);
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

#endif
