
#define IMU_SDA 26
#define IMU_SCL 27

#define IMU_I2C i2c1

void imu_init();

void imu_read(double *roll, double *pitch, double *yaw);