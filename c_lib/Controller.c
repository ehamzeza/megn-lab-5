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

// static float last_input = 0.0f;
// static float last_output = 0.0f;

float Controller_Update( Controller_t* p_cont, float measurement, float dt )
{
    // float B[2] = {3.962409525759, -3.545764150520};
    // float A[2] = {1.000000000000, -0.583354624761};

    // float B[2] = 
    // {
    //     rb_get_F(&p_cont->controller.numerator, 0),
    //     rb_get_F(&p_cont->controller.numerator, 1)
    // };
    // float A[2] = 
    // {
    //     rb_get_F(&p_cont->controller.denominator, 0),
    //     rb_get_F(&p_cont->controller.denominator, 1)
    // };
    
    // float now = ((B[0] * measurement) + (B[1] * last_input) - (A[1] * last_output)) / A[0];
    // last_input = measurement;
    // last_output = now;
    float now = Filter_Value(&p_cont->controller, measurement);


    float pwm_right = p_cont->kp * (p_cont->target_pos - now);

    // struct {
    //     float v1;
    // } data;
    // data.v1 = rb_get_F(&p_cont->controller.numerator, 0);
    // USB_Send_Msg( "cf", 'B', &data, sizeof( data ) );

    return Saturate(pwm_right, 400.0f);

    // float filter_output = Filter_Value(&p_cont->controller, measurement);

    // // p_cont->target_pos += p_cont->target_vel * dt;

    // float error = p_cont->target_pos - filter_output;
    // float unconstrained = p_cont->kp * error;
    // float saturated = Saturate(unconstrained, 400.0f);

    // return saturated;
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

