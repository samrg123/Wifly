#pragma once

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class Imu: public Adafruit_MPU6050 {

  public:

    static const char* AccelerationRangeString(mpu6050_accel_range_t range) {
      
      switch(range) {
        case MPU6050_RANGE_2_G:  return "MPU6050_RANGE_2_G";
        case MPU6050_RANGE_4_G:  return "MPU6050_RANGE_4_G";
        case MPU6050_RANGE_8_G:  return "MPU6050_RANGE_8_G";
        case MPU6050_RANGE_16_G: return "MPU6050_RANGE_16_G";
        default:                 return "INVALID";
      }
    }

    static const char* GyroRangeString(mpu6050_gyro_range_t range) {
      
      switch(range) {
          case MPU6050_RANGE_250_DEG:  return "MPU6050_RANGE_250_DEG";
          case MPU6050_RANGE_500_DEG:  return "MPU6050_RANGE_500_DEG";
          case MPU6050_RANGE_1000_DEG: return "MPU6050_RANGE_1000_DEG";
          case MPU6050_RANGE_2000_DEG: return "MPU6050_RANGE_2000_DEG";
          default:                     return "INVALID";
      }
    }

    void PrintStatus() {
      Serial.printf(
        "MPU6050 Status: {\n"
        "\tAcceleration Range: %s\n"
        "\tGryo Range: %s\n"
        "\tBandwidth: %s\n"
        "}\n",

        AccelerationRangeString(getAccelerometerRange()),
        GyroRangeString(getGyroRange()),
        BandwidthString(getFilterBandwidth())
      );
    }

    static const char* BandwidthString(mpu6050_bandwidth_t bandwidth) {
      
      switch(bandwidth) {
        case MPU6050_BAND_260_HZ: return "MPU6050_BAND_260_HZ";
        case MPU6050_BAND_184_HZ: return "MPU6050_BAND_184_HZ";
        case MPU6050_BAND_94_HZ:  return "MPU6050_BAND_94_HZ";
        case MPU6050_BAND_44_HZ:  return "MPU6050_BAND_44_HZ";
        case MPU6050_BAND_21_HZ:  return "MPU6050_BAND_21_HZ";
        case MPU6050_BAND_10_HZ:  return "MPU6050_BAND_10_HZ";
        case MPU6050_BAND_5_HZ:   return "MPU6050_BAND_5_HZ";
        default:                  return "INVALID";
      }      
    }
    
    void Init(mpu6050_accel_range_t accelerationRange = MPU6050_RANGE_8_G, 
              mpu6050_gyro_range_t  gyroRange = MPU6050_RANGE_500_DEG,
              mpu6050_bandwidth_t   bandwidth = MPU6050_BAND_5_HZ) {
      
      // Try to initialize
      while(!begin()) {
        Serial.println("Failed to find MPU6050 chip | Retrying");      
        delay(500);
      }
          
      setAccelerometerRange(accelerationRange);
      setGyroRange(gyroRange);
      setFilterBandwidth(bandwidth);

      PrintStatus();
    }

};
