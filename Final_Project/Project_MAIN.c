/**
 * lab 7 parts
 */
#include "open_interface.h"
#include "movement.h"
#include "lcd.h"
//#include "cyBot_Scan.h"
#include <string.h>
#include "button.h"
#include "adc.h"
#include "Timer.h"
#include "uart.h"
#include "ping.h"
#include "servo.h"
#include "navigation.h"
#include "music.h"

typedef struct {
	float ping_dist;
	int IR_raw_value;
} scan_t;

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

// movement
extern short obstacle_error;
//extern short hit_2_obstacles;

int main(void)
{
    timer_init();
    lcd_init();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    // music
    // load_songs();

    // Initialize UART and UART interrupts to receive characters.
    uart_init();
    uart_interrupt_init();

    // initialize buttons
    button_init();
    init_button_interrupts();

    // Initialize ADC/IR
    adc_init();
    // Initialize ping sensor
    ping_initialize();

    // Initialize servo
    servo_init();
//    servo_calibrate();

    // Apply left-right calibration values.
    right_calibration_value = 6823;
    left_calibration_value = 35827;

    // calibrate IR sensor
    //IR_calibrate();

    // Upon detecting a border or obstacle, trigger a flag that we have to manually clear by pressing o.
    // Also, it will send a message back indicating what type of obstacle and what it did to correct

	char data_buffer[50];  // Buffer to store data
	stop = 0;
	int angle_traversed = 0;
	int distance_traversed = 0;
	obstacle_error = 0;
	lcd_printf("Running");
	//uart_sendString("Manual Drive\r\n");

	while (1) {
		switch (data_char) {
			case 'w':
				move_with_detection(sensor_data, 5, 150);
				if (obstacle_error == 1) {
				    distance_traversed = 0;
				    //uart_sendString("Please correct for obstacle error.\r\n");
				}
				else {
				    angle_traversed = 0;
				    distance_traversed += 5;
				    sprintf(data_buffer, "Distance Driven: %d cm\r\n", distance_traversed);
				    uart_sendString(data_buffer);
				}
				data_char = 0;
				break;
			case 'a': // turning left is represented as negative
				angle_traversed -= 5;
				distance_traversed = 0;
				turn_left(sensor_data, 5);
				sprintf(data_buffer, "Degrees Turned: %d\r\n", angle_traversed);
				uart_sendString(data_buffer);
				data_char = 0;
				break;
			case 's': // Reversed
				if (obstacle_error != 1) {
					angle_traversed = 0;
					distance_traversed -= 5;
					reverse(sensor_data, 5, 250);
					sprintf(data_buffer, "Distance Driven: %d cm\r\n", distance_traversed);
					uart_sendString(data_buffer);
				}
				data_char = 0;
				break;
			case 'd': // turning right is represented as positive
				angle_traversed += 5;
				distance_traversed = 0;
				turn_right(sensor_data, 5);
				sprintf(data_buffer, "Degrees Turned: %d\r\n", angle_traversed);
				uart_sendString(data_buffer);
				data_char = 0;
				break;

			case 'q': // turn 45 degrees left (negative)
				angle_traversed -= 45;
				distance_traversed = 0;
				turn_left(sensor_data, 41);
				sprintf(data_buffer, "Degrees Turned: %d\r\n", angle_traversed);
				uart_sendString(data_buffer);
				data_char = 0;
				break;
			case 'e': // turn 45 degrees right (positive)
				angle_traversed += 45;
				distance_traversed = 0;
				turn_right(sensor_data, 42);
				sprintf(data_buffer, "Degrees Turned: %d\r\n", angle_traversed);
				uart_sendString(data_buffer);
				data_char = 0;
				break;

			case 'o': // clear obstacle flag and total traversed
				angle_traversed = 0;
				distance_traversed = 0;
				obstacle_error = 0;
				data_char = 0;
				uart_sendString("Block cleared.");
				break;
			case 'c': // scan for objects
				IR_and_ping_scan();
				data_char = 0;
				break;
			case '_':
				uart_sendString("Open the pod bay doors Toonces!\r\n");
				data_char = 0;
				break;
		}

	}

    oi_free(sensor_data);
    return 0;
}

