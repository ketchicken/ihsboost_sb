#include "kipr/wombat.h"
#include "sbl.hpp"
#include "config.hpp"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

int LEFT_COE = 1700;
int RIGHT_COE = 1700;

/**
 * stop the motors completely to make sure they do not drift after finishing the rotation 
*/  
void brake(){
    mav(LEFT_MOTOR, 0);
    mav(RIGHT_MOTOR, 0);
    msleep(1);
}
  
/**
 * @brief automatically calibrate the ticks each wheel has
 * The function adjusts the COE of each wheel until the appropriate amount of degrees has been turnt
*/ 

void calib_gmpc(vector<int> tests){
    //int degrees[2] = {90, 180}; //all the degrees that need testing
    int l_coe = 0; //change in accordance with amt of tests
    int r_coe = 0;
    // Calibrate left wheel first
    for(vector<int>::iterator iterator = tests.begin(); iterator < tests.end(); ++iterator){
        //Calibrate left wheel
        cmpc(LEFT_MOTOR);
        gyro_turn_degrees({0,0}, {900, 0}, *iterator);
        brake();
        msleep(10);
        l_coe += (int)(gmpc(LEFT_MOTOR) / ((WHEEL_DISTANCE/2) * RADIANS(*iterator) / WHEEL_CIRCUMFERENCE));

        //Calibrate Right Wheel
        cmpc(RIGHT_MOTOR);

        gyro_turn_degrees({0,0}, {0, 900}, (*iterator)*-1);
        brake();
        msleep(10);

        r_coe += (int)(gmpc(RIGHT_MOTOR) / ((WHEEL_DISTANCE/2) * RADIANS(*iterator) / WHEEL_CIRCUMFERENCE));

		cout << "Test for " << *iterator << " done" << endl;
        
    }
    msleep(2000);
    //test out coefficients 
    RIGHT_COE = r_coe/(int)tests.size();
    LEFT_COE = l_coe/(int)tests.size();
    
    //put RIGHT and LEFT in files
    write_gmpc();
    
}

void write_gmpc(){
    Json::Value jobj;
    ifstream _if("small_bot_config.json");
    _if >> jobj;

    jobj["gmpc_right_coefficient"] = RIGHT_COE;
    jobj["gmpc_left_coefficient"] = LEFT_COE;
}

/**
 * stop for a certain amount of time in ms
 * @param time the amount of time to stop in ms
*/
void stop_for_ms(int time){
    ao();
    msleep(time);
}

/**
 * turn the robot with the left wheel being the pivot point (turn using right wheel)
 * @param degrees degrees the robot has to turn, left is negative, right is positive
 * @param speed speed of the robot when turning, range: 1-1500
*/
void turn_l_pivot(int degrees, unsigned int speed){
    cmpc(RIGHT_MOTOR);
    int target = -(int)((WHEEL_DISTANCE/2)*RADIANS(degrees)/WHEEL_CIRCUMFERENCE * RIGHT_COE); //(r*theta)/wheel_circumference to get the amount of roations 
    if(target > 0){ //left turn
        while(gmpc(RIGHT_MOTOR) < target) mav(RIGHT_MOTOR, speed*RIGHT_BIAS);
        brake();
    }
    else if(target < 0){ //right turn
        while(gmpc(RIGHT_MOTOR) > target) mav(RIGHT_MOTOR, -speed*RIGHT_BIAS);
        brake();
    }
}

/**
 * turn the robot with the right wheel being the pivot point (turn using the left wheel)
 * @param degrees degrees the robot has to turn, left is negative, right is positive
 * @param speed speed of the robot when turning, range: 1-1500
*/
void turn_r_pivot(int degrees, unsigned int speed){
    cmpc(LEFT_MOTOR);
    int target = (int)((WHEEL_DISTANCE/2)*RADIANS(degrees)/WHEEL_CIRCUMFERENCE * LEFT_COE);
    if(target < 0){ //left turn
        while(gmpc(LEFT_MOTOR) > target) mav(LEFT_MOTOR, -speed*LEFT_BIAS);
        brake();
    }
    else if(target > 0){ //right turn 
        while(gmpc(LEFT_MOTOR) < target) mav(LEFT_MOTOR, speed*LEFT_BIAS);
        brake();
    }
}

/**
 * turn the robot with the center being the pivot point
 * @param degrees degrees the robot has to turn, left is negative, right is positive 
 * @param speed speed of the robot when turning, range: 1-1500
*/
void spin(int degrees, unsigned int speed){
    cmpc(LEFT_MOTOR);
    cmpc(RIGHT_MOTOR);
    int left_target = (int)((WHEEL_DISTANCE/2)*RADIANS(degrees)/WHEEL_CIRCUMFERENCE * LEFT_COE/2);
    int right_target = -(int)((WHEEL_DISTANCE/2)*RADIANS(degrees)/WHEEL_CIRCUMFERENCE * RIGHT_COE/2);
    if(degrees < 0){ //left turn
        while(gmpc(LEFT_MOTOR) > left_target || gmpc(RIGHT_MOTOR) < right_target){
            if(gmpc(LEFT_MOTOR) > left_target) mav(LEFT_MOTOR, -speed*LEFT_BIAS);
            else if(gmpc(LEFT_MOTOR) <= left_target) mav(LEFT_MOTOR, 0);
            if(gmpc(RIGHT_MOTOR) < right_target) mav(RIGHT_MOTOR, speed*RIGHT_BIAS);
            else if(gmpc(RIGHT_MOTOR) >= right_target) mav(RIGHT_MOTOR, 0);
        } 
        brake();
    }
    else if(degrees > 0){ //right turn 
        while(gmpc(LEFT_MOTOR) < left_target || gmpc(RIGHT_MOTOR) > right_target){
            if(gmpc(LEFT_MOTOR) < left_target) mav(LEFT_MOTOR, speed*LEFT_BIAS);
            else if(gmpc(LEFT_MOTOR) >= left_target) mav(LEFT_MOTOR, 0);
            if(gmpc(RIGHT_MOTOR) > right_target) mav(RIGHT_MOTOR, -speed*RIGHT_BIAS);
            else if(gmpc(RIGHT_MOTOR) <= right_target) mav(RIGHT_MOTOR, 0);
        }
        brake();
    }
}

/**
 * go straight using motor tick counter
 * not suitable for long distances
 * @param displacement distance the robot needs to travel, forward is positive, backward is negative
 * @param speed speed of the robot, range: 1-1500
*/
void straight_gmpc(int displacement, unsigned int speed){
    cmpc(LEFT_MOTOR);
    cmpc(RIGHT_MOTOR);
    int left_target = (int)(displacement/WHEEL_CIRCUMFERENCE*(displacement/abs(displacement))*LEFT_COE); //# of roations*sign(displacement)
    int right_target = (int)(displacement/WHEEL_CIRCUMFERENCE*(displacement/abs(displacement))*RIGHT_COE);
    if(displacement > 0){ //forward
        while(gmpc(LEFT_MOTOR) < left_target || gmpc(RIGHT_MOTOR) < right_target){
            if(gmpc(LEFT_MOTOR) < left_target) mav(LEFT_MOTOR, speed*RIGHT_BIAS);
            else if(gmpc(LEFT_MOTOR) >= left_target) mav(LEFT_MOTOR, 0);
            if(gmpc(RIGHT_MOTOR) < right_target) mav(RIGHT_MOTOR, speed*RIGHT_BIAS);
            else if(gmpc(RIGHT_MOTOR) <= right_target) mav(RIGHT_MOTOR, 0);
        }
        brake();
    }
    else if(displacement < 0){ //backward
        while(gmpc(LEFT_MOTOR) > left_target || gmpc(RIGHT_MOTOR) > right_target){
            if(gmpc(LEFT_MOTOR) > left_target) mav(LEFT_MOTOR, -speed*LEFT_BIAS);
            else if(gmpc(LEFT_MOTOR) <= left_target) mav(LEFT_MOTOR, 0);
            if(gmpc(RIGHT_MOTOR) > right_target) mav(RIGHT_MOTOR, -speed*RIGHT_BIAS);
            else if(gmpc(RIGHT_MOTOR) <= right_target) mav(RIGHT_MOTOR, 0);
        }
        brake();
    }
}

void pid_align(int target_1, int target_2, int offset, int side, std::function<bool()> stop_condition, int updates_per_second, double kp, double ki, double kd){
    PIDController speed_1 = PIDController(kp, ki, kd, updates_per_second); 
    PIDController speed_2 = PIDController(kp, ki, kd, updates_per_second);

    while( !stop_condition() && ((LEFT_TOPHAT > target_1 + offset || LEFT_TOPHAT < target_1 - offset) || (RIGHT_TOPHAT > target_2 + offset || RIGHT_TOPHAT < target_2 - offset))){
        speed_1.step(LEFT_TOPHAT, target_1);
        speed_2.step(RIGHT_TOPHAT, target_2);

        MOVEMENT_FUNCTION((int)speed_1.speed() * side, (int)speed_2.speed() * side);
        msleep(1000/updates_per_second);
        brake();
    }

} 