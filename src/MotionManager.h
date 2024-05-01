#pragma once
#ifndef MOTIONMANAGER_H
#define MOTIONMANAGER_H

#include <functional>
#include <map>
#include <vector>
// #include <Adafruit_ICM20X.h>
// #include <Adafruit_BNO055.h>
// #include <Adafruit_ICM20948.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ICM_20948.h>

#define kTestTwirlBPM 0 // 0 for off, 5 is slowish, 17 is enough to trigger rotation overlays

typedef enum : unsigned int {
  JumpActivity,
  ActivityTypeCount,
} ActivityType;

typedef std::function<void()> ActivityHandler;
typedef std::map<const char *, ActivityHandler> ActivityHandlerMap;

struct Euler {
  float pitch,roll,yaw; // FIXME: convert to integer math
};

class MotionManager {
private:
  bool hasSensor = false;
  unsigned int retainCount;
  std::vector<ActivityHandlerMap> activityHandlers;

  // std::map<Adafruit_BNO055::adafruit_vector_type_t, sensors_event_t> eventMap;
  
  // inline void cacheEventType(Adafruit_BNO0::adafruit_vector_type_t type) {
  //   sensors_event_t event;
  //   bno.getEvent(&event, type);
  //   eventMap[type] = event;
  // }
  ICM_20948_I2C icm;
  void initDMP() {
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
  MotionManager() {
  }

  bool init(TwoWire *wire=&Wire) {
    logf("mm INIT");
    icm.begin(Wire, 0);
    // hasSensor = icm.begin_I2C(0b1101000);
    hasSensor = (icm.status == ICM_20948_Stat_Ok);
    logf("ICM20948 init = %i", hasSensor);
    if (hasSensor) {
      initDMP();
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

  void accelerationAtPixelIndex(PixelIndex index) {
    Point pos = ledgeometry.position(index); // in um

  }

  ICM_20948_AGMT_t agmt = {0};

  void loop() {
    if (!hasSensor) {
      return;
    }
    agmt = icm.getAGMT();
    
    // eventMap.clear(); // new frame new events
    // twirlCached = false;

    // sensors_event_t accel = event(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // sensors_event_t linear_accel = event(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // logf("accel, lin_accel: (%0.3f, %0.3f)", accel.acceleration.x, linear_accel.acceleration.x);


    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
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

  // accelerationAtPoint()
  /*
  Ok, let's apply. Acceleration (Ax, Ay, Az) and gyroscope (Gx, Gy, Gz) are sampled in the local (moving) reference frame. 
  The distance (r_p) and angle (theta_p) from the sensor to the center of the disc is also known. 
  Write pseudo code to compute the acceleration experienced at point Q on the disc.


  # Define required functions
def cross_product(v1, v2):
    """ Calculate cross product of two vectors v1 and v2 """
    return [v1[1]*v2[2] - v1[2]*v2[1], 
            v1[2]*v2[0] - v1[0]*v2[2], 
            v1[0]*v2[1] - v1[1]*v2[0]]

def rotate_vector(v, theta):
    """ Rotate vector v by angle theta in the plane """
    return [v[0] * cos(theta) - v[1] * sin(theta),
            v[0] * sin(theta) + v[1] * cos(theta),
            v[2]]

# Inputs
Ax, Ay, Az = 0.0, 0.0, 0.0  # Acceleration at P (local frame)
Gx, Gy, Gz = 0.0, 0.0, 0.0  # Gyroscope readings at P (angular velocity in rad/s)
r_p, theta_p = 0.0, 0.0     # Distance and angle from sensor P to center C
r_q, theta_q = 0.0, 0.0     # Distance and angle from point Q to center C

# Convert polar coordinates to cartesian in the disc's local frame
x_p = r_p * cos(theta_p)
y_p = r_p * sin(theta_p)
x_q = r_q * cos(theta_q)
y_q = r_q * sin(theta_q)

# Angular velocity vector
omega = [Gx, Gy, Gz]

# Tangential velocity at P due to rotation
v_p_tan = cross_product(omega, [x_p, y_p, 0])

# Rotational acceleration contributions at P
a_p_rot = [- (Gx**2 + Gy**2 + Gz**2) * x_p, - (Gx**2 + Gy**2 + Gz**2) * y_p, 0]
a_p_rot = [a_p_rot[i] + 2 * cross_product(omega, v_p_tan)[i] for i in range(3)]

# Calculate center acceleration
a_c = [Ax - a_p_rot[0], Ay - a_p_rot[1], Az - a_p_rot[2]]

# Calculate acceleration at Q
v_q_tan = cross_product(omega, [x_q, y_q, 0])  # Tangential velocity at Q due to rotation
a_q_rot = [- (Gx**2 + Gy**2 + Gz**2) * x_q, - (Gx**2 + Gy**2 + Gz**2) * y_q, 0]
a_q_rot = [a_q_rot[i] + 2 * cross_product(omega, v_q_tan)[i] for i in range(3)]
a_q = [a_c[0] + a_q_rot[0], a_c[1] + a_q_rot[1], a_c[2] + a_q_rot[2]]

# Output acceleration at Q in the local frame
print("Acceleration at Q:", a_q)


  */

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

#endif
