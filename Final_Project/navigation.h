#include "open_interface.h"
#include "movement.h"
#include "lcd.h"
#include <string.h>
#include "button.h"
#include "uart.h"
#include "servo.h"
#include "adc.h"
#include "ping.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

typedef struct {
    short obj_number;
    float distance;
    int start_angle;
    int end_angle;
} object_t;

typedef struct {
	int heading;
	float distance;
	signed char turn_direction; // -1 is right, +1 is left, 0 means no objects were detected and the bot will proceed ahead
	char plan_pending; // 0 is false, 1 means plan has not been executed
} movePlan_t;


/**
 * Runs a program to get calibration data for the IR sensor.
 * Collects 10 scans when a button is pressed and sends them to PuTTy.
 * Procedure is to start at 10cm and work back in 2cm increments.
 */
void IR_calibrate();

/**
 * Scan with the IR sensor and our functions.
 */
int scan_IR(float degrees);

/**
 * Ping scan with our functions.
 */
float scan_ping(float degrees);

/**
 * Scan to detect objects and return the smallest object.
 */
void IR_and_ping_scan();

void display_cliff_values(oi_t *sensor);

void random_drive(oi_t *sensor, int speed);

#endif
