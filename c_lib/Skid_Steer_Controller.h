/*
         MEGN540 Mechatronics Lab
    Copyright (C) Andrew Petruska, 2022.
       apetruska [at] mines [dot] edu
          www.mechanical.mines.edu
*/

/*
    Copyright (c) 2022 Andrew Petruska at Colorado School of Mines

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.

*/

/*
 * This file defines a skid-steer controller for the zumo-car. This system
 * will take in a desired linear and angular velocity for the car, convert these
 * to desired left and right side wheel speeds, and have the controller update.
 */
#include "Controller.h"
#include "SerialIO.h"

typedef struct {
    Controller_t controller_left;   // Z-Transform Controller for the left-side drive
    Controller_t controller_right;  // Z-Transform Controller for the right-side drive

    float wheel_base_width;             // The left-to-right seporation distance between the two side's drive wheels
    float conversion_speed_to_control;  // Relates the linear tangential speed of the left-side or right-side wheel to the motor speed being controlled
    float max_abs_control;              // The maximum control that can be applied, to enable control saturation

    //// Consider adding in the following functionality to implement slew-limited commands
    // float max_lin_accel;
    // float max_ang_accel;
    // float max_lin_vel;
    // float max_ang_vel;
    // float lin_pos_target_request;
    // float lin_pos_target_current;
    // float ang_pos_target_request;
    // float ang_pos_target_current;
    // float lin_vel_target_request;
    // float lin_vel_target_current;
    // float ang_vel_target_request;
    // float ang_vel_target_current;
    // uint8_t velocity_mode;

    float ( *measurement_left_fcn_ptr )( void );   // function pointer to a function that provides a measurement for the left-side's drive angular measurement
    float ( *measurement_right_fcn_ptr )( void );  // function pointer to a function that provides a measurement for the right-side's drive angular measurement

    void ( *control_left_fcn_ptr )( int16_t );   // function pointer to a function that alows a left-side drive control to be specified
    void ( *control_right_fcn_ptr )( int16_t );  // function pointer to a function that alows a right-side drive control to be specified

} Skid_Steer_Controller_t;

/**
 * @brief Initialize_Skid_Steer initializes the skid steer objet
 *
 * @param p_skid_steer_cntr a pointer to the object being initialized
 * @param z_transform_numerator the controller's z-transform numerator
 * @param z_transform_denominator the controller's z-transform denominator
 * @param z_transform_order the controller's order
 * @param wheel_base_width the axel-width between treds
 * @param conversion_speed_to_control the conversion from m/s to control units (pwm?)
 * @param max_abs_control the absolute maximum control for satruation in control units (pwm?)
 * @param measurement_left_fcn_ptr a function pointer to the left-side measurement
 * @param measurement_right_fcn_ptr a function pointer to the right-side measurement
 * @param control_left_fcn_ptr a function pointer to the left side's control application
 * @param control_right_fcn_ptr a founction pointer to the right side's control applicaion
 */
void Initialize_Skid_Steer( Skid_Steer_Controller_t* p_skid_steer_cntr, float* z_transform_numerator, float* z_transform_denominator, uint8_t z_transform_order,
                            float wheel_base_width, float conversion_speed_to_control, float max_abs_control, float ( *measurement_left_fcn_ptr )( void ),
                            float ( *measurement_right_fcn_ptr )( void ), void ( *control_left_fcn_ptr )( int16_t ), void ( *control_right_fcn_ptr )( int16_t ) );

/**
 * @brief Skid_Steer_Command_Displacement sets a new target diplacment for the robot to execute. This is a relative displacment to the current position, not an
 * absolute target.
 *
 * @param linear The arc-length to travel (m)
 * @param angular The angle to rotate (rad)
 */
void Skid_Steer_Command_Displacement( Skid_Steer_Controller_t* p_skid_steer_cntr, float linear, float angular );

/**
 * @brief Skid_Steer_Command_Velocity sets a target velocity for the skid-steer system to execute
 *
 * @param linear The tangential velocity to move at (m/s)
 * @param angular The angular rate to spin at (rad/s)
 */
void Skid_Steer_Command_Velocity( Skid_Steer_Controller_t* p_skid_steer_cntr, float linear, float angular );

/**
 * @brief Skid_Steer_Control_Update executes a control update by comparing current measurments desired values and implements new control setpoints.
 *
 */
void Skid_Steer_Control_Update( Skid_Steer_Controller_t* p_skid_steer_cntr, float elapsed_time );
