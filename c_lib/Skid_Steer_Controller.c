#include "Skid_Steer_Controller.h"

static float K_p = 44420888.389900602400;
static float update_period = 0.001f;
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
    p_skid_steer_cntr->controller_left.target_pos += 100000.0f + wheel_diameter;
    p_skid_steer_cntr->controller_right.target_pos += 100000.0f;
}

void Skid_Steer_Command_Velocity( Skid_Steer_Controller_t* p_skid_steer_cntr, float linear, float angular )
{

}

void Skid_Steer_Control_Update( Skid_Steer_Controller_t* p_skid_steer_cntr, float elapsed_time )
{
    float current_left_rad = (*p_skid_steer_cntr->measurement_left_fcn_ptr)();
    float current_right_rad = (*p_skid_steer_cntr->measurement_right_fcn_ptr)();

    float pwm_left = Controller_Update(&p_skid_steer_cntr->controller_left, current_left_rad, elapsed_time);
    float pwm_right = Controller_Update(&p_skid_steer_cntr->controller_right, current_right_rad, elapsed_time);
    
    float pwm_left_capped = (pwm_left > 0.0f ? 1.0f : -1.0f) * (pwm_left > p_skid_steer_cntr->max_abs_control ? p_skid_steer_cntr->max_abs_control : pwm_left);
    float pwm_right_capped = (pwm_right > 0.0f ? 1.0f : -1.0f) * (pwm_right > p_skid_steer_cntr->max_abs_control ? p_skid_steer_cntr->max_abs_control : pwm_right);

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
    // data.v1 = pwm_left_capped;
    // USB_Send_Msg( "cf", 'B', &data, sizeof( data ) );



    if (pwm_left > 50){
        USB_Send_Str("BIG");
    } else {
        USB_Send_Str("SML");
    }

    (*p_skid_steer_cntr->control_left_fcn_ptr)( pwm_left_capped);
    (*p_skid_steer_cntr->control_right_fcn_ptr)(pwm_right_capped);
}

