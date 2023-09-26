

#ifndef MPU6000_H_
#define MPU6000_H_

// registers:
#define MPU6000_ACCEL_READ 0x3B|0x80
#define MPU6000_TEMP_READ  0x41|0x80
#define MPU6000_GYRO_READ  0x43|0x80
#define MPU6000_WHO_I_AM_READ 0x75|0x80
#define MPU6000_PRODUCT_ID_READ  0x0C|0x80
#define MPU6000_FIFO_READ   0x74|0x80


void setup_MPU6000();
void MPU6000_SPI_write(uint8_t adress_of_register, uint8_t value);
void MPU6000_SPI_read(uint8_t adress_of_register, uint8_t* data, uint16_t size);
void MPU6000_SPI_read_DMA(const uint8_t adress_of_register, uint8_t* data, uint16_t size);
void gyro_update(timeUs_t time);
void acc_update(timeUs_t time);
void gyro_calibration(gyro_t* gyro_to_calibrate, timeUs_t time);
uint8_t acc_calibration(acc_t* acc_to_calibrate);
void acc_level_calibration(acc_t* acc_to_calibration);
void read_acc();
void read_gyro();
void read_temp();
void read_all();
void read_all_DMA();

#endif /* MPU6000_H_ */
