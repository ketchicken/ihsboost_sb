#ifndef SBL_HPP
#define SBL_HPP

#include "controllers.hpp"
//#include "sb_defines.hpp"
#include "timer.hpp"
#include "imu_movement.hpp"

/* robot attributes */
#ifndef LEFT_MOTOR
#define LEFT_MOTOR 0 //left motor port
#endif
#ifndef RIGHT_MOTOR
#define RIGHT_MOTOR 1 //right motor port
#endif

#ifndef LEFT_TOPHAT 
#define LEFT_TOPHAT analog(1)
#endif
#ifndef RIGHT_TOPHAT
#define RIGHT_TOPHAT analog(2)
#endif

#define LEFT_BIAS 1 //left motor speed bias
#define RIGHT_BIAS 1 //right motor speed bias

#define WHEEL_DISTANCE 2 
#define WHEEL_CIRCUMFERENCE 3.14

#define RADIANS(deg) (deg*M_PI/180) //degrees to radians macro  

#define average2(first, second) ((first + second)/2) // find the average of two numbers

void brake();
void calib_gmpc(int tests[]);
void write_gmpc();
void stop_for_ms(int time_in_milliseconds);
void turn_l_pivot(int degrees, unsigned int speed);
void turn_r_pivot(int degrees, unsigned int speed);
void spin(int degrees, unsigned int speed); //ccw is negative, cw is positive degrees
void straight_gmpc(int displacement, unsigned int speed);

/**
 * @brief align until both tophats are on target values 
 * 
 * @param target_1 desired target value for left tophat sensor
 * @param target_2 desired target value for right tophat sensor
 * @param offset amount of error the tophat can have
 * @param side which side of the tape to align on (bot is on black tape = 1, bot is approaching black tape = -1)
 * @param updates_per_second amount of time the PID controller updates per second, defaults to 200
 * @param kp coefficient of p
 * @param ki coefficient of i
 * @param kd coefficient of d
*/
void pid_align(int target_1, int target_2, int offset, int side, std::function<bool()> stop_condition=Timer(4.99), int updates_per_second = 200, double kp = 0.5, double ki = 1, double kd = -0.001); //port 1 is generally left, port 2 is right

#endif