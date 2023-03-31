#include "Lab5_Tasks.h"

void Task_Control_Loop(float time_since_last)
{
    Skid_Steer_Control_Update(&car_controller, time_since_last);
}