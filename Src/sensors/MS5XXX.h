
#ifndef MS5XXX_H_
#define MS5XXX_H_

/**
 *@brief library for MS56XX, MS57XX, MS58XX modules. Implemented only for I2C
 *@note Address of the devise is different models. You have to change MS5XXX_ADDR for your personal aplication. Some of addresses are provided maybre your sensor is below.
 * Also you need to provide I2C functions used for communication (defined in I2C1.h) if needed.
 */

 /**
  * Different sensors (models) have different addresses so you need to update MS5XXX_ADDR for your personal device.
  * Some addresses are provifed for youe convinience (feel free to add more)
  * MS5611-01BA03 has address dependent on CSB pin value so you can use 2 same sensors. If CSB == 1 -> MS5XXX_ADDR = 0xEE or CSB == 0 -> MS5XXX_ADDR = 0xEC
  * datasheet: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036
  * MS5837-30BA MS5XXX_ADDR = 0x76
  * datasheet: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5837-30BA%7FB1%7Fpdf%7FEnglish%7FENG_DS_MS5837-30BA_B1.pdf%7FCAT-BLPS0017
 */



#define MS5XXX_SENSOR MS5611 //  MS5611 MS5837   


#define MS5XXX_ADDR 0xEE  // modify if needed
#define MS5XXX_ADDR_W MS5XXX_ADDR // Module address write mode
#define MS5XXX_ADDR_R MS5XXX_ADDR|0x01 // Module address read mode
#define MS5XXX_CMD_RESET 0x1E // ADC reset command
#define MS5XXX_CMD_ADC_READ 0x00 // ADC read command
#define MS5XXX_CMD_ADC_CONV 0x40 // ADC conversion command
#define MS5XXX_CMD_ADC_D1 0x00 // ADC D1 conversion
#define MS5XXX_CMD_ADC_D2 0x10 // ADC D2 conversion
#define MS5XXX_CMD_ADC_256 0x00 // ADC OSR=256
#define MS5XXX_CMD_ADC_512 0x02 // ADC OSR=512
#define MS5XXX_CMD_ADC_1024 0x04 // ADC OSR=1024
#define MS5XXX_CMD_ADC_2048 0x06 // ADC OSR=2048
#define MS5XXX_CMD_ADC_4096 0x08 // ADC OSR=4096
#define MS5XXX_CMD_PROM_RD 0xA0 // Prom read command (first coefficient, next is 0xA2, up to 0xAE)
#define MS5XXX_CONVERSION_TIME 600 // time [us] needed for conversion for 265 version (it is more-less proportional so for 4096 you need 16-times longer time than for 256)

typedef struct {
    char* name;
    // coefficients which you've once read from PROM and keep for ever:
    uint16_t C1_SENS;
    uint16_t C2_OFF;
    uint16_t C3_TCS;
    uint16_t C4_TCO;
    uint16_t C5_T;
    uint16_t C6_TEMPSENS;
    uint32_t ADC_value[2];
    float temperature;
    float raw_preasure;
    float filtered_preasure;
    float h0_preasure;
    float altitude;
    float vel_raw;
    float ver_vel;
}baro_t;


void MS5XXX_reset();
void MS5XXX_calculate_preasure(baro_t* baro);
void MS5XXX_PROM_read(uint8_t* data);
void MS5XXX_start_conversion_preasure(uint8_t CONVERSION_OSR);
void MS5XXX_start_conversion_temp(uint8_t CONVERSION_OSR);
void MS5XXX_read_preasure(baro_t* baro);
void MS5XXX_read_temp(baro_t* baro);
void MS5XXX_ADC_read_wait(baro_t* baro, uint8_t CONVERSION_OSR);
bool MS5XXX_init(baro_t* baro);
void MS5XXX_unstack();

#endif /* MS5XXX_H_ */
