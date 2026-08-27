/*
 * Sensor.h
 *
 * Created: 2026-08-27 오후 7:53:59
 *  Author: kccistc
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

void Sensors_Init(void);
void Sensors_Task(void);

// IR Sensor
void Sensors_IrClear(void);
int Sensors_IrDetected(void);
void Sensors_IrEnable(bool en);

// Home Sensor
int Sensors_HomeLevel(AxisId axis);
int Sensors_HomeLatched(AxisId axis);
void Sensors_HomeClear(AxisId axis);


#endif /* SENSOR_H_ */