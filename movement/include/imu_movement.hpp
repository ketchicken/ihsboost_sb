#ifndef GYRO_MOVEMENT_HPP
#define GYRO_MOVEMENT_HPP

#include "speed.hpp"
#include <functional>

#define GYRO_LINEAR_ACCEL 500
#define GYRO_UPDATES_PER_SEC 200
#define GYRO_CORRECTION_PROPORTION .90
#define GYRO_TURN_V2_MIN_SPEED 10

#define ACCEL_MIN_SPEED 10
#define ACCEL_LINEAR_ACCEL 500
#define ACCEL_UPDATES_PER_SEC 200
#define ACCEL_CORRECTION_PROPORTION .90

/**
 * @brief Drive the create straight using the gyroscope
 *
 * @param from_speed speed to start at
 * @param to_speed speed to cap at
 * @param stop_function a function that returns true when it is time to stop driving forward
 * @param correction_proportion how much the correction is by; defaults to .90,
 * should be between (0, 1)
 * @param accel_per_sec how fast the create will accelerate by, defaults to 500
 * @param updates_per_sec how many updates the function will do per sec, defaults to 200
 */
void gyro_drive_straight(int from_speed, int to_speed, std::function<bool()> stop_function, double correction_proportion = GYRO_CORRECTION_PROPORTION, double accel_per_sec = GYRO_LINEAR_ACCEL, int updates_per_sec = GYRO_UPDATES_PER_SEC);

/**
 * @brief Turn the create a certain number of degrees using the gyroscope
 *
 * @param from_speed the speed to start at
 * @param to_speed the speed to end at
 * @param degrees how many degrees to turn (+ vals for CW, - vals for CCW)
 * @param accel_per_sec how fast to accelerate, defaults to 500
 * @param updates_per_sec how many updates the function will do per sec, defaults to 200
 */
void gyro_turn_degrees(Speed from_speed, Speed to_speed, int degrees, double accel_per_sec = GYRO_LINEAR_ACCEL, int updates_per_sec = GYRO_UPDATES_PER_SEC);

/**
 * @brief Turns the create, starting at rest and ending at rest, and turning at max at max_speed
 *
 * @param max_speed the maximum speed to turn at, use a positive value
 * @param degrees how many degrees to turn (+ vals for CW, - vals for CCW)
 * @param min_speed the minimum speed to turn at, use a positive value, defaults to 10
 * @param accel_per_sec how fast to accelerate, defaults to 500
 * @param updates_per_sec how many updates the function will do per sec, defaults to 200
 */
void gyro_turn_degrees_v2(int max_speed, int degrees, int min_speed = GYRO_TURN_V2_MIN_SPEED, double accel_per_sec = GYRO_LINEAR_ACCEL, int updates_per_sec = GYRO_UPDATES_PER_SEC);

void accel_drive_cm(int max_speed, double cm, int min_speed = ACCEL_MIN_SPEED, double accel_per_sec = ACCEL_LINEAR_ACCEL, double correction_proportion = ACCEL_CORRECTION_PROPORTION, int updates_per_sec = ACCEL_UPDATES_PER_SEC);
#endif