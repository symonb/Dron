
#ifndef BAROMETER_H_
#define BAROMETER_H_

void baro_calculate_altitude(baro_t* baro, timeUs_t current_time);
void baro_kalman_fusion(baro_t* baro, timeUs_t current_time);
void baro_kalman_fusion_v2(baro_t* baro, timeUs_t current_time);
void baro_set_h0_preasure(baro_t* baro);
void alt_hold(timeUs_t dt);
#endif /* BAROMETER_H_ */