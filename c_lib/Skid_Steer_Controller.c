#include "Skid_Steer_Controller.h"

static float wheel_diameter = 35.95 / 1000.0f;

void Initialize_Skid_Steer( Skid_Steer_Controller_t* p_skid_steer_cntr, float* z_transform_numerator, float* z_transform_denominator, uint8_t z_transform_order, float update_period,
                            float wheel_base_width, float conversion_speed_to_control, float max_abs_control, float ( *measurement_left_fcn_ptr )( void ),
                            float ( *measurement_right_fcn_ptr )( void ), void ( *control_left_fcn_ptr )( int16_t ), void ( *control_right_fcn_ptr )( int16_t ) )
{
    Initialize_Controller(&p_skid_steer_cntr->controller_left, conversion_speed_to_control, z_transform_numerator, z_transform_denominator, z_transform_order, update_period);
    Initialize_Controller(&p_skid_steer_cntr->controller_right, conversion_speed_to_control, z_transform_numerator, z_transform_denominator, z_transform_order, update_period);

    Controller_SetTo(&p_skid_steer_cntr->controller_left, 0.0f);
    Controller_SetTo(&p_skid_steer_cntr->controller_right, 0.0f);

    p_skid_steer_cntr->wheel_base_width = wheel_base_width;
    p_skid_steer_cntr->conversion_speed_to_control = conversion_speed_to_control;
    p_skid_steer_cntr->max_abs_control = max_abs_control;

    p_skid_steer_cntr->measurement_left_fcn_ptr = measurement_left_fcn_ptr;
    p_skid_steer_cntr->measurement_right_fcn_ptr = measurement_right_fcn_ptr;
    p_skid_steer_cntr->control_left_fcn_ptr = control_left_fcn_ptr;
    p_skid_steer_cntr->control_right_fcn_ptr = control_right_fcn_ptr;
}

void Skid_Steer_Command_Displacement( Skid_Steer_Controller_t* p_skid_steer_cntr, float linear, float angular )
{
    p_skid_steer_cntr->controller_left.target_pos += (linear - angular * (p_skid_steer_cntr->wheel_base_width / 1.0f)) * (2.0f / wheel_diameter);
    p_skid_steer_cntr->controller_right.target_pos += (linear + angular * (p_skid_steer_cntr->wheel_base_width / 1.0f)) * (2.0f / wheel_diameter);

    p_skid_steer_cntr->controller_left.target_vel = 0.0f;
    p_skid_steer_cntr->controller_right.target_vel = 0.0f;
}

void Skid_Steer_Command_Velocity( Skid_Steer_Controller_t* p_skid_steer_cntr, float linear, float angular )
{
    p_skid_steer_cntr->controller_left.target_vel = linear - angular;
    p_skid_steer_cntr->controller_right.target_vel = linear + angular;

    p_skid_steer_cntr->controller_left.target_pos = (*p_skid_steer_cntr->measurement_left_fcn_ptr)();
    p_skid_steer_cntr->controller_right.target_pos = (*p_skid_steer_cntr->measurement_right_fcn_ptr)();
}

void Skid_Steer_Stop( Skid_Steer_Controller_t* p_skid_steer_cntr)
{
    p_skid_steer_cntr->controller_left.target_vel = 0.0f;
    p_skid_steer_cntr->controller_right.target_vel = 0.0f;

    p_skid_steer_cntr->controller_left.target_pos = (*p_skid_steer_cntr->measurement_left_fcn_ptr)();
    p_skid_steer_cntr->controller_right.target_pos = (*p_skid_steer_cntr->measurement_right_fcn_ptr)();
}

void Skid_Steer_Control_Update( Skid_Steer_Controller_t* p_skid_steer_cntr, float elapsed_time )
{

    float current_left_rad = (*p_skid_steer_cntr->measurement_left_fcn_ptr)();
    float current_right_rad = (*p_skid_steer_cntr->measurement_right_fcn_ptr)();

    float pwm_left = Controller_Update(&p_skid_steer_cntr->controller_left, current_left_rad, elapsed_time);
    float pwm_right = Controller_Update(&p_skid_steer_cntr->controller_right, current_right_rad, elapsed_time);

    (*p_skid_steer_cntr->control_left_fcn_ptr)((int16_t) ((int) pwm_left));
    (*p_skid_steer_cntr->control_right_fcn_ptr)((int16_t) ((int) pwm_right));
}

