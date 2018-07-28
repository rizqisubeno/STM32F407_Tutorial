#include "stm32f4xx_hal.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t mpuAddress;
extern uint8_t Mmode;
extern uint8_t Mscale;
extern uint8_t Gscale;
extern uint8_t Ascale;
extern uint8_t Mmode;
extern float aRes, gRes, mRes;
extern int16_t accelCount[3];
extern int16_t gyroCount[3];
extern int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
extern float magCalibration[3],magbias[3];  // Factory mag calibration and mag bias
extern float gyroBias[3],accelBias[3]; // Bias corrections for gyro and accelerometer
extern float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
extern int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
extern float temperature;
extern float SelfTest[6];

extern int delt_t; // used to control display output rate
extern int count;  // used to control display output rate
extern float pitch, yaw, roll;
extern float deltat;                             // integration interval for both filter schemes
extern int lastUpdate, firstUpdate, Now;    // used to calculate integration interval                               // used to calculate integration interval
extern float q[4];           // vector to hold quaternion
extern float eInt[3];


void writeByte(uint8_t mpuAddressddress, uint8_t subAddress, uint8_t data);
int readByte(uint8_t mpuAddress,uint8_t subAddress);
void readBytes(uint8_t mpuAddress, uint8_t subAddress,uint8_t count, uint8_t *dest);

void getMres(void);
void getGres(void);
void getAres(void);

void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void readMagData(int16_t * destination);
int16_t readTempData(void);

void resetMPU9250(void);
void initAK8963(float * destination);
void initMPU9250(void);
void calibrateMPU9250(float * dest1,float *dest2);
void MPU9250SelfTest(float * destination);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
