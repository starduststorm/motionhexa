#pragma once
#ifndef MOTIONMANAGER_H
#define MOTIONMANAGER_H

#include <functional>
#include <map>
#include <vector>
#include <Wire.h>
#include <ICM_20948.h>

#include "hexaphysics.h"

#define kTestTwirlBPM 0 // 0 for off, 5 is slowish, 17 is enough to trigger rotation overlays

typedef enum : unsigned int {
  JumpActivity,
  ActivityTypeCount,
} ActivityType;

typedef std::function<void()> ActivityHandler;
typedef std::map<const char *, ActivityHandler> ActivityHandlerMap;

class MotionManager {
protected:
    static MotionManager *_singleton;
    MotionManager() {}
    const bool enableDMP = false;
public:
    MotionManager(MotionManager &other) = delete;
    void operator=(const MotionManager &) = delete;
    static MotionManager &manager();

private:
  bool hasSensor = false;
  unsigned int retainCount;
  std::vector<ActivityHandlerMap> activityHandlers;

  ICM_20948_I2C icm;
  void initDMP() {
    logf("Init DMP...");
    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);

    // Enable any additional sensors / features
    //success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    //success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (icm.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (icm.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (icm.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (icm.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (icm.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (icm.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (icm.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success) {
      Serial.println(F("DMP enabled!"));
    } else {
      Serial.println(F("Enable DMP failed!"));
    }
  }

public:

  bool init(TwoWire *wire=&Wire) {
    logf("motionManager INIT");
    icm.begin(Wire, 0);
    hasSensor = (icm.status == ICM_20948_Stat_Ok);
    logf("  ICM20948 init = %i", hasSensor);
    if (hasSensor) {
      if (enableDMP) {
        initDMP();
      }
      for (unsigned i = 0; i < ActivityTypeCount; ++i) {
        activityHandlers.push_back(ActivityHandlerMap());
      }
    }
    return hasSensor;
  }

  void addActivityHandler(ActivityType activity, const char *identifier, ActivityHandler handler) {
    activityHandlers[activity][identifier] = handler;
  }

  void removeActivityHandler(ActivityType activity, const char *identifier) {
    activityHandlers[activity].erase(identifier);
  }

  // sensors_event_t event(Adafruit_BNO055::adafruit_vector_type_t type) {
  //   if (eventMap.find(type) == eventMap.end()) {
  //     // memoize for this frame
  //     cacheEventType(type);
  //   }
  //   return eventMap[type];
  // }
  vector16 accelerationAtPixelIndex(PixelIndex index) {
    // imu_pos = 8.0506, 22.9692 # 108.0506, 77.0308 relative to 100,100 center
    UMPoint P = UMPoint::fromMM(8.0506, -22.9692); // FIXME: can this be constexpr?
    UMPoint Q = hexGrid.position(index); // in um
    vector16 accel(-agmt.acc.axes.x, agmt.acc.axes.y);
    // vector16 gyro(agmt.gyr.axes.x, agmt.gyr.axes.y);
    // gyro = gyro / 1000;
    // gyro = gyro.scale8(0x02);
    // logf("accelerationAtPixelIndex(%03i), Q=(%i,%i), accel=(%i,%i), gryo=(%i, %i)", index, Q.x, Q.y, accel.x, accel.y, gyro.x, gyro.y);

    // FIXME: doesn't work, oversimplified

    UMPoint P2Q = Q - P;
    if (index == 0 || index == 270) {
      // logf("index %i P2Q = (%i, %i)", index, P2Q.x, P2Q.y);
    }
    auto ω_z = agmt.gyr.axes.z / 15000;
    auto vec = vector16(accel.x + ω_z*ω_z * P2Q.x, accel.y + ω_z*ω_z * P2Q.y);
    // logf("  => (%i, %i)", vec.x, vec.y);
    return vec;
  }

  ICM_20948_AGMT_t agmt = {0};

  void loop() {
    if (!hasSensor) {
      return;
    }
    agmt = icm.getAGMT();
    if (!enableDMP) {
      return;
    }

    icm_20948_DMP_data_t data;
    icm.readDMPdataFromFIFO(&data);

    if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
      //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
      //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
      //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
      //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
      //SERIAL_PORT.println( data.header, HEX );

      if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
      {
        // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
        // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
        // The quaternion data is scaled by 2^30.

        //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

        // Scale to +/- 1
        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

        // Convert the quaternions to Euler angles (roll, pitch, yaw)
        // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

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

        // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
        // Serial.print(F("{\"quat_w\":"));
        // Serial.print(q0, 3);
        // Serial.print(F(", \"quat_x\":"));
        // Serial.print(q1, 3);
        // Serial.print(F(", \"quat_y\":"));
        // Serial.print(q2, 3);
        // Serial.print(F(", \"quat_z\":"));
        // Serial.print(q3, 3);
        // Serial.println(F("}"));

        // logf("Pitch: %0.3f, roll: %0.3f, yaw: %0.3f", pitch, roll, yaw);
      }
    }
  }

private:
  float twirlVelocityAccum;
  float prevXOrientation;
  bool twirlCached = false;
public:
//   float twirlVelocity(int samples=10, float *outOrientation=NULL) {
//     float orientation;

// #if kTestTwirlBPM != 0
//     orientation = (beatsin16(kTestTwirlBPM, 0, 1000) - 500) / 2;
// #else
//     sensors_event_t euler = event(Adafruit_BNO055::VECTOR_EULER);
//     orientation = euler.orientation.x;
// #endif
//     if (outOrientation) {
//       *outOrientation = orientation;
//     }
//     // repeated calls in the same frame should not update twirl accum
//     if (twirlCached) {
//       return twirlVelocityAccum;
//     }
//     twirlCached = true;
//     twirlVelocityAccum = (samples * twirlVelocityAccum + MOD_DISTANCE(prevXOrientation, orientation, 360)) / (samples + 1);
//     prevXOrientation = orientation;
//     return twirlVelocityAccum;
//   }
};

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)  {
    Serial.print("-");
  }  else  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      Serial.print("0");
    }    else    {
      break;
    }
  }
  if (val < 0) {
    Serial.print(-val, decimals);
  }  else  {
    Serial.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  Serial.print(" ]");
  Serial.println();
}

MotionManager *MotionManager::_singleton = nullptr;
MotionManager &MotionManager::manager() {
    if (_singleton==nullptr) {
        _singleton = new MotionManager();
    }
    return *_singleton;
}

#endif
