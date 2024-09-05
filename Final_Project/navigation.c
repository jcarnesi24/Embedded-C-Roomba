#include "navigation.h"

// buttons
extern volatile int button_event;
extern volatile int button_num;

// Uart
extern volatile char data_char;
extern volatile char stop;

// ping
extern volatile enum status current_status;
extern volatile int rising_edge_time;
extern volatile int falling_edge_time;
extern int num_overflows;

// servo
extern int right_calibration_value;
extern int left_calibration_value;

void IR_calibrate() {
	// the Roomba has to be ON to use the sensors
	//servo_move(90); // ensure the scanner is facing the right direction.
	int IR_total = 0;
	short scan_size = 8; // was 10
	char data_buffer[8];
	short i = 0;
	short scan_real_distance = 10; // starts at 10 cm
	lcd_printf("Calibrating_IR");
	while(1) {
		if (button_num) {
			for (i = 0; i < scan_size; ++i) {
				IR_total += adc_read();
			}
			IR_total /= scan_size;

			sprintf(data_buffer, "%d,%u\r\n", IR_total, scan_real_distance);
			scan_real_distance += 2;

			uart_sendString(data_buffer);
			lcd_clear();
			lcd_puts(data_buffer);
			button_num = 0;
			timer_waitMillis(1000);
		}
	}
}

int scan_IR(float degrees) {
	servo_move(degrees);
	timer_waitMillis(60);
	// example distance formula: IR_distance = 1 [varies] * pow(10, 6 [varies]) * pow((IR_raw_sum / (float)num_scans), -1.49 [varies]);
	return adc_read();
}

float scan_ping(float degrees) {
	servo_move(degrees);
	timer_waitMillis(60);
	return ping_read() * 0.0010625;
}

void IR_and_ping_scan() {
    int current_angle = 0; // currently scanned angle
    int starting_angle = 0; // starting angle of an object
    float previous_distance = 100; // a dummy previous distance so our algorithm works
    char scanning_object = 0; // true or false value
    short object_num = 0; // number of objects found
    object_t found_objects[10]; // array of object_t's for the objects found
    char data_buffer[35]; // data sent to PuTTy
    short i = 0; // iterator
    short num_scans = 3; // number of scans per angle
    int IR_raw_sum = 0; // sum of raw IR values
	float IR_distance = 0; // averaged raw IR value converted to cm using formula
	float obj_distance = 0; // distance to an object in the struct
	int obj_mid_angle = 0; // angle to scan the middle of an object


	uart_sendString("Degrees\t\tIR Distance (cm)\n\r");

	servo_move(0);
	timer_waitMillis(150);
    // perform scan
   while (current_angle <= 180 && !stop) {
	   IR_raw_sum = 0;
	   IR_distance = 0;
	   for (i = 0; i < num_scans; i++) {
		   //cyBOT_Scan(current_angle, &current_reading);
		   IR_raw_sum += scan_IR(current_angle);
	   }

       // Convert IR raw value into an averaged distance in centimeters using
	   // the line of best fit for the calibration of that robot.
	   IR_distance = 877514 * pow((IR_raw_sum / (float)num_scans), -1.45); // cybot 1

	   sprintf(data_buffer, "%d,%.00f,\n\r", current_angle, IR_distance);
	   uart_sendString(data_buffer);
       // The cyBot can only provide useful data at distances under 50 cm.
       // if we get a reading under 50 cm, save that angle as a potential object.
       if (scanning_object) {
    	   // If no longer scanning object
           if (abs(IR_distance - previous_distance) > 10 || current_angle == 180) {
        	   // and the object was at least 4 degrees of travel (throw out bad readings)
               if (abs(starting_angle - current_angle) > 4) { // was 14
            	   // the robot found an object
            	   obj_mid_angle = (starting_angle + current_angle) / 2.0;
            	   servo_move(obj_mid_angle);
            	   timer_waitMillis(100);

                   found_objects[object_num].start_angle = starting_angle;
                   found_objects[object_num].end_angle = current_angle;
                   found_objects[object_num].obj_number = object_num + 1;
                   // Scan middle of object with ping sensor to obtain distance
                   for (i = 0; i < num_scans; i++) {
					   obj_distance += scan_ping(obj_mid_angle);
				   }
                   servo_move(current_angle);
                   found_objects[object_num].distance = obj_distance / num_scans;
                   obj_distance = 0;
                   ++object_num;
               }
               scanning_object = 0;

           }
       } else { // we could detect a new object
           // check if we are detecting a new object
           // If the change is also more than 10 centimeters, we have a new object. (save the previously detected object)
           if (IR_distance < 70 /*&& abs(IR_distance - previous_distance) > 10*/) {
               starting_angle = current_angle;
               scanning_object = 1;
           }
       }


       previous_distance = IR_distance;

       current_angle += 2;
   }

   uart_sendString("Object#\tAngle\tDistance (cm)\tWidth (deg)\tLinear Width (cm)\n\r");

   float linear_width = 0;
   int width;
   // go through all the objects and find the smallest one
   for (i = 0; i < object_num; i++) {
       width = found_objects[i].end_angle - found_objects[i].start_angle;
       linear_width = 2.0 * found_objects[i].distance * fabs(sin(0.5 * (width * M_PI / 180.0)));
       sprintf(data_buffer, "%u\t,%d,\t%.00f,\t\t%d,\t\t%.00f,\n\r", found_objects[i].obj_number, found_objects[i].start_angle, found_objects[i].distance, width, linear_width);
       uart_sendString(data_buffer);
   }

   uart_sendString("OBJECT COMPLETE\n");

   servo_move(0);
}

void display_cliff_values(oi_t *sensor) {
	while (1) {
	    	oi_update(sensor);
	    	// white will send sensor values > 2700
	    	// black is < 1200
	    	// actual hole is < 120

	    	lcd_printf("FL:%d, FR:%d\nL:%d, R:%d", sensor->cliffFrontLeftSignal, sensor->cliffFrontRightSignal,
					   sensor->cliffLeftSignal, sensor->cliffRightSignal);
	    	timer_waitMillis(1000);
	    }
}

void random_drive(oi_t *sensor, int speed) {
	char data_buffer[50];
	sprintf(data_buffer, "L\tFL\tFR\tR\n\r");
	uart_sendString(data_buffer);
	oi_setWheels(speed, speed);
	double time = 600000; // 10 minutes
	double elapsed_time = 0;
	while (elapsed_time < time) {
		oi_update(sensor);
//    	sprintf(data_buffer, "%d\t%d\t%d\t%d\n\r", sensor_data->cliffLeftSignal, sensor_data->cliffFrontLeftSignal,
//					sensor_data->cliffFrontRightSignal, sensor_data->cliffRightSignal);
//    	uart_sendString(data_buffer);
		if (sensor->bumpLeft){
		   left_impact(sensor);
		   //distance_traveled -= 150;
		   oi_setWheels(speed, speed);
	   }

	   if (sensor->bumpRight){
		   right_impact(sensor);
		   //distance_traveled -= 150;
		   oi_setWheels(speed, speed);
	   }
	   cliff_data_processor(sensor);
//		   if (hit_2_obstacles == 1) {
//			   break;
//		   }
	   oi_setWheels(speed, speed);
	   elapsed_time += timer_getMillis();
	}
	oi_setWheels(0, 0);
}
