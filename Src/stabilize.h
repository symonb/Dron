/*
 * stabilize.h
 *
 *  Created on: 04.01.2021
 *      Author: filip
 */

#ifndef STABILIZE_H_
#define STABILIZE_H_

void att_update(timeUs_t dt_us);
void stabilize(timeUs_t dt_us);
void send_telemetry_stabilize(timeUs_t time);

#endif /* STABILIZE_H_ */
