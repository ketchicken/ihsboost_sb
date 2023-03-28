#include <kipr/wombat.h>
#include <ihsboost/all.hpp>
#include <iostream>
using namespace std;

//Define the two tophats you will be using for the alignment
#define LEFT_TOPHAT analog(1)
#define RIGHT_TOPHAT analog(2)

int main()
{
    // Left target, right target, max error of 10, go backwards/forwards, max time spent is 5 seconds, updates per second, Kp, Ki, Kd
    pid_align(3310, 3180, 10, 1, Timer(5), 200, 0.5, 1, -0.001); //example of aligning on the space in front of the tophat sensors
  
    //the last 5 parameters default to Timer(5), 200, 0.5, 1, -0.001
    pid_align(2600, 2500, 100, -1) //example of aligning on space behind the tophat sensors
      
    ao();
    return 0;
}
