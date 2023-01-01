//=============== IMU sensor ===============
/* Configure gyroscope and accelerometer */
#include <basicMPU6050.h> 
                      
//----------------------- Settings -------------------------

// Gyro settings:
#define         LP_FILTER   6           // Low pass filter.                    Value from 0 to 6  -> 5hz cutoff
#define         GYRO_SENS   1           // Gyro sensitivity.                   Value from 0 to 3  -> +-250 deg/s
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3  -> +-4g
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.  

// Accelerometer offset:
constexpr int   AX_OFFSET = 0;       // Adjust these values so the accelerometer outputs (0,0,1)g if held level. 
constexpr int   AY_OFFSET = 0;       // The z-axis must be calibrated for the flight controller to work properly.
constexpr int   AZ_OFFSET = 0; 

//----------------------------------------------------------

// Set parameters:
basicMPU6050<LP_FILTER , GYRO_SENS , ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET , AY_OFFSET , AZ_OFFSET
             > imu;

//----------------------------------------------------------

// Rotated outputs [adjust depending on sensor orientation]
/* The gyroscope axes may not be aligned the axes of the aircraft. 
 * To circumvent this problem, the sensor axes can be swapped to 
 * match roll, pitch and yaw axes, (x, y, z) respectively. 
*/

float gyroX() { return  imu.gy(); } 
float gyroY() { return -imu.gz(); }
float gyroZ() { return  imu.gx(); }

// sign of accelerometer axes may differ from gyroscope 
float accelX() { return -imu.ay(); }
float accelY() { return  imu.az(); }
float accelZ() { return  imu.ax(); }
