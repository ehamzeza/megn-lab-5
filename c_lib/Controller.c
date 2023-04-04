#include "Controller.h"


void Initialize_Controller(Controller_t* p_cont, float kp, float* num, float* den, uint8_t order, float update_period)
{
    p_cont->kp = kp;
    p_cont->update_period = update_period;

    p_cont->target_pos = 0.0f;
    p_cont->target_vel = 0.0f;

    Filter_Init(&p_cont->controller, num, den, order);
    Filter_SetTo(&p_cont->controller, 0.0f);
}

void Controller_Set_Target_Velocity( Controller_t* p_cont, float vel )
{
    p_cont->target_vel = vel;
}

void Controller_Set_Target_Position( Controller_t* p_cont, float pos )
{
    p_cont->target_pos = pos;
    p_cont->target_vel = 0.0f;
}

float Controller_Update( Controller_t* p_cont, float measurement, float dt )
{
    float now = Filter_Value(&p_cont->controller, measurement);

    float pwm = p_cont->kp * (p_cont->target_pos - now);

    // p_cont->target_pos += dt * p_cont->target_vel;
    if (p_cont->target_vel != 0)
        p_cont->target_pos = measurement + (p_cont->target_vel * dt * 0.0001);

    return Saturate(pwm, 400.0f);
}

float Controller_Last( Controller_t* p_cont)
{
    return Filter_Last_Output(&p_cont->controller);
}

void Controller_SetTo(Controller_t* p_cont, float measurement )
{
    Filter_SetTo(&p_cont->controller, measurement);
}

void Controller_ShiftBy(Controller_t* p_cont, float measurement )
{
    Filter_ShiftBy(&p_cont->controller, measurement);
}

