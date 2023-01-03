//============= Sensor fusion ==============
/* IMU sensor fusion filter - horizontal estimate */
#include <imuFilter.h>

//----------------------- Settings -------------------------

#define GAIN          0.2     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector */

#define SD_ACCEL      0.3     /* Standard deviation of acceleration. Accelerations relative to (0,0,1)g outside of this band are suppresed */                          

//----------------------------------------------------------
imuFilter fusion;

void setupFusion() {
  fusion.setup( accelX(), accelY(), accelZ() ); 
}
    
void updateFusion() {  
  fusion.update( gyroX(), gyroY(), gyroZ(), 
                 accelX(), accelY(), accelZ(), 
                 GAIN, SD_ACCEL );  
}
