#include "controllers.hpp"
#include "servos.hpp"
#include <kipr/wombat.h>

void move_servo_slowly(int port, int position, int speed, int updates_per_sec)
{
    LinearController accelerator(get_servo_position(port), position, speed, updates_per_sec);

    enable_servo(port);
    while (!accelerator.done())
    {
        // first only in this case because it's a case where you need to
        // have msleeps before the final one
        accelerator.step();

        set_servo_position(port, static_cast<int>(accelerator.speed()));
        msleep(accelerator.get_msleep_time());
    }
    set_servo_position(port, position);
    disable_servo(port);
}

void servo(int port, int position, int step){
    enable_servo(port);
    while(get_servo_position(port) < position){
        set_servo_position(port, get_servo_position(port) + step);
        msleep(step);
    }
    while(get_servo_position(port) > position){
        set_servo_position(port, get_servo_position(port) - step);
        msleep(step);
    }
    set_servo_position(port, position);
    disable_servo(port);
}