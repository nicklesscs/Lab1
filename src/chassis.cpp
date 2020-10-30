#include <Romi32U4.h>
#include "chassis.h"

float RomiChassis::SpeedLeft(void)
{
 
  return (C_wheel* (count_left- prev_count_left))/(N_wheel*( interval/ 1000)); //Angular velocity *radius = tangential velocity
  //*1000 convert to second

    // !!! ATTENTION !!! Angular velocity * radius = tangential velocity
    // Assignment 1
    
}

float RomiChassis::SpeedRight(void)
{
  
  return (C_wheel* (count_right- prev_count_right))/(N_wheel*(interval/1000)); //Angular velocity *radius = tangential velocity
  
  //*1000 convert to second
    // !!! ATTENTION !!!
    // Assignment 1
    
}

void RomiChassis::UpdateEffortDriveWheels(int left, int right)
{ 
    motors.setEfforts(left,right);
    chassis.SerialPlotter(SpeedLeft(), millis(), SpeedRight(), millis()); //print left and right speed at current time
}

void RomiChassis::UpdateEffortDriveWheelsPI(int target_speed_left, int target_speed_right)
{
  //ki
  E_left += (target_speed_left-(C_wheel*(encoders.getCountsLeft()- prev_count_left)/(N_wheel*( interval/ 1000))));  // find accu error on the left wheel, use encoder to find the current left count and get left speed
  E_right += (target_speed_right-(C_wheel*(encoders.getCountsRight()- prev_count_right)/(N_wheel*( interval/ 1000)))); // same as above

  float u_left =Kp * (target_speed_left-(C_wheel*(encoders.getCountsLeft()- prev_count_left)/(N_wheel*( interval/ 1000)))) + Ki *E_left; //kp part + ki part
  float u_right = Kp * (target_speed_right-(C_wheel*(encoders.getCountsRight()- prev_count_right)/(N_wheel*( interval/ 1000)))) + Ki * E_right;//right wheel

  motors.setEfforts(u_left,u_right);
  prev_count_left = encoders.getCountsLeft();
  prev_count_right = encoders.getCountsRight();
  chassis.SerialPlotter(SpeedLeft(),E_left, SpeedRight(), E_right);

  //E_left = target_speed_left - SpeedLeft()
  //error = target_speed_right - SpeedRight()
  // !!! ATTENTION !!!
  // Assignment 2
}

void RomiChassis::SerialPlotter(float a, float b, float c, float d)
{
    // !!! ATTENTION !!!
    // USE this function for assignment 3!
    
    Serial.print(a);
    Serial.print('\t');
    Serial.print(b);
    Serial.print('\t');
    Serial.print(c);
    Serial.print('\t');
    Serial.print(d);
    Serial.println();

}

void RomiChassis::MotorControl(void)
{
  uint32_t now = millis();
  if(now - last_update >= interval)
  {    
    prev_count_left = count_left;
    prev_count_right = count_right;
    count_left = encoders.getCountsLeft();
    count_right = encoders.getCountsRight();
    previous_time = millis();
    UpdateEffortDriveWheels(target_left, target_right);
    last_update = now;
  }
}

void RomiChassis::StartDriving(float left, float right, uint32_t duration)
{
  target_left = left; target_right = right;
  start_time = millis();
  last_update = start_time;
  end_time = start_time + duration; //fails at rollover
  E_left = 0;
  E_right = 0;
}

bool RomiChassis::CheckDriveComplete(void)
{
  return millis() >= end_time;
}

void RomiChassis::Stop(void)
{
  target_left = target_right = 0;
  motors.setEfforts(0, 0);
}