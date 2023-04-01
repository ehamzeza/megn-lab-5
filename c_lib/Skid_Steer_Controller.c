#include "Skid_Steer_Controller.h"

static float K_p = 93.883642643262;
static float update_period = 0.002f;
static float wheel_diameter = 38.5 / 1000.0f;

void Initialize_Skid_Steer( Skid_Steer_Controller_t* p_skid_steer_cntr, float* z_transform_numerator, float* z_transform_denominator, uint8_t z_transform_order,
                            float wheel_base_width, float conversion_speed_to_control, float max_abs_control, float ( *measurement_left_fcn_ptr )( void ),
                            float ( *measurement_right_fcn_ptr )( void ), void ( *control_left_fcn_ptr )( int16_t ), void ( *control_right_fcn_ptr )( int16_t ) )
{
    Initialize_Controller(&p_skid_steer_cntr->controller_left, K_p, z_transform_numerator, z_transform_denominator, z_transform_order, update_period);
    Initialize_Controller(&p_skid_steer_cntr->controller_right, K_p, z_transform_numerator, z_transform_denominator, z_transform_order, update_period);

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
    // p_skid_steer_cntr->controller_left.target_pos += (linear - angular * (p_skid_steer_cntr->wheel_base_width / 2.0f)) * (2.0f / wheel_diameter);
    // p_skid_steer_cntr->controller_right.target_pos += (linear + angular * (p_skid_steer_cntr->wheel_base_width / 2.0f)) * (2.0f / wheel_diameter);
    p_skid_steer_cntr->controller_left.target_pos += 2.0 * 3.14f * 2.0f + (0 * wheel_diameter);
    p_skid_steer_cntr->controller_right.target_pos += 2.0 * 3.14f * 2.0f;
}

void Skid_Steer_Command_Velocity( Skid_Steer_Controller_t* p_skid_steer_cntr, float linear, float angular )
{

}

static float last_input = 0.0f;
static float last_output = 0.0f;

void Skid_Steer_Control_Update( Skid_Steer_Controller_t* p_skid_steer_cntr, float elapsed_time )
{

    // float current_left_rad = (*p_skid_steer_cntr->measurement_left_fcn_ptr)();
    float current_right_rad = (*p_skid_steer_cntr->measurement_right_fcn_ptr)();
    
    // float pwm_right = Controller_Update(&p_skid_steer_cntr->controller_right, current_right_rad, elapsed_time);
    float Kp = 93.883642643262f;
    float B[2] = {3.962409525759, -3.545764150520};
    float A[2] = {1.000000000000, -0.583354624761};

    float now = ((B[0] * current_right_rad) + (B[1] * last_input) - (A[1] * last_output)) / A[0];

    last_output = now;
    last_input = current_right_rad;


    int16_t pwm_right = Kp * (p_skid_steer_cntr->controller_right.target_pos - now);
    if (pwm_right < -400.0f) pwm_right = -400.0f;
    if (pwm_right > 400.0f) pwm_right = 400.0f;

    (*p_skid_steer_cntr->control_right_fcn_ptr)((int16_t) pwm_right);

    // if (p_skid_steer_cntr->controller_right.target_pos < current_right_rad)
    // {
    //     (*p_skid_steer_cntr->control_right_fcn_ptr)(-65);
    // } else {
    //     (*p_skid_steer_cntr->control_right_fcn_ptr)(65);
    // }

    // float pwm_left = Controller_Update(&p_skid_steer_cntr->controller_left, current_left_rad, elapsed_time);
    // float pwm_right = Controller_Update(&p_skid_steer_cntr->controller_right, current_right_rad, elapsed_time);
    
    // float pwm_left_capped = (pwm_left > 0.0f ? 1.0f : -1.0f) * (pwm_left > p_skid_steer_cntr->max_abs_control ? p_skid_steer_cntr->max_abs_control : pwm_left);
    // float pwm_right_capped = (pwm_right > 0.0f ? 1.0f : -1.0f) * (pwm_right > p_skid_steer_cntr->max_abs_control ? p_skid_steer_cntr->max_abs_control : pwm_right);

    // struct {
    //     float left;// = pwm_left;
    //     float right;// = pwm_right;
    // } data;
    // data.left = pwm_left;
    // data.right = pwm_right;

    // USB_Send_Msg("cff", 'i', &data, sizeof(data));
    // struct {
    //     float v1;
    // } data;
    // data.v1 = current_left_rad;
    // USB_Send_Msg( "cf", 'B', &data, sizeof( data ) );

    // int16_t final_left = (int16_t) pwm_left_capped;
    // int16_t final_right = (int16_t) pwm_right_capped;
    

    // (*p_skid_steer_cntr->control_left_fcn_ptr)(final_left);
    // (*p_skid_steer_cntr->control_right_fcn_ptr)(final_right);
}

