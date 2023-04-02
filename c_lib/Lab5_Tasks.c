#include "Lab5_Tasks.h"

void Task_Control_Loop(float time_since_last)
{
    Skid_Steer_Control_Update(&car_controller, time_since_last);
}


void Task_Stop_Control(float _not_used)
{
    Skid_Steer_Stop(&car_controller);
    Task_Cancel(&task_stop_control);
}