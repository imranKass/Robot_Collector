/*
 * Robot_Arm.c
 *
 *  Created on: Dec 4, 2025
 *      Author: imran
 */
/**
 * @file Robot_Arm.c
 *
 * @brief Source code for the Pololu BKT18A Robot Arm driver using Timer A1.
 *
 * This file contains the function implementations for controlling the BKT18A robot arm
 * using Timer A1 on pins P7.4, P7.5, and P7.6.
 *
 * @author Your Name
 * @date December 2025
 */

#include "../inc/Robot_Arm.h"
#include "../inc/Clock.h"

/* PWM Configuration for 50Hz servo control
 * System Clock: 3 MHz
 * Desired PWM Frequency: 50 Hz (20ms period)
 * Timer Period: 3,000,000 Hz / 50 Hz = 60,000 counts
 */
#define PWM_PERIOD          60000   // 20ms period at 3MHz

// Height servo range (lift servo) - 1000µs to 1900µs
#define HEIGHT_PULSE_MIN    3000    // 1000µs (fully raised)
#define HEIGHT_PULSE_MAX    5700    // 1900µs (fully lowered)

// Tilt servo range (pivot servo) - 1200µs to 1900µs
#define TILT_PULSE_MIN      3600    // 1200µs (fully down)
#define TILT_PULSE_MAX      5700    // 1900µs (fully up)

// Gripper servo range (paddle servo) - 500µs to 2400µs
#define GRIPPER_PULSE_MIN  1500    // 500µs (fully open)
#define GRIPPER_PULSE_MAX   7200    // 2400µs (fully closed)

// Current servo positions - START AT MIDDLE (90°) for testing
static uint8_t current_height_angle = 60;
static uint8_t current_tilt_angle = 90;
static uint8_t current_gripper_angle = 10;  // Gripper starts at 10° (safe for your assembly)

/**
 * @brief Convert angle (0-180) to PWM duty cycle value for height servo
 */
static uint16_t Angle_To_PWM_Height(uint8_t angle)
{
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }
    return HEIGHT_PULSE_MIN +
           ((uint32_t)angle * (HEIGHT_PULSE_MAX - HEIGHT_PULSE_MIN)) / SERVO_MAX_ANGLE;
}

/**
 * @brief Convert angle (0-180) to PWM duty cycle value for tilt servo
 */
static uint16_t Angle_To_PWM_Tilt(uint8_t angle)
{
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }
    return TILT_PULSE_MIN +
           ((uint32_t)angle * (TILT_PULSE_MAX - TILT_PULSE_MIN)) / SERVO_MAX_ANGLE;
}

/**
 * @brief Convert angle (0-180) to PWM duty cycle value for gripper servo
 */
static uint16_t Angle_To_PWM_Gripper(uint8_t angle)
{
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }
    return GRIPPER_PULSE_MIN +
           ((uint32_t)angle * (GRIPPER_PULSE_MAX - GRIPPER_PULSE_MIN)) / SERVO_MAX_ANGLE;
}

void Robot_Arm_Init(void)
{
    // Configure P7.4, P7.5, P7.6 as Timer A1 outputs
    P7->SEL0 |= 0x70;   // Set bits 4, 5, 6
    P7->SEL1 &= ~0x70;  // Clear bits 4, 5, 6
    P7->DIR |= 0x70;    // Set as outputs

    // Configure Timer A1 for PWM generation
    TIMER_A1->CCR[0] = PWM_PERIOD - 1;

    // DIAGNOSTIC: Initialize servos to MIDDLE positions for visible movement

    current_height_angle=60;
    current_tilt_angle=90;
    current_gripper_angle=60;

    // HEIGHT servo to 90° (middle position)
    TIMER_A1->CCTL[4] = 0x00E0;
    TIMER_A1->CCR[4] = Angle_To_PWM_Height(current_height_angle);

    // TILT servo to 90° (level position)
    TIMER_A1->CCTL[2] = 0x00E0;
    TIMER_A1->CCR[2] = Angle_To_PWM_Tilt(current_tilt_angle);

    // GRIPPER servo to 10° (slightly closed from fully open)
    TIMER_A1->CCTL[3] = 0x00E0;
    TIMER_A1->CCR[3] = Angle_To_PWM_Gripper(current_gripper_angle);

    // Start Timer A1
    TIMER_A1->CTL = 0x0294; //TIMER_A1->CTL = 0x0214;

    // Track current positions

}

void Robot_Arm_Set_Servo_Angle(Servo_Type servo, uint8_t angle)
{
    uint16_t pwm_value;

    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }

    switch(servo) {
        case HEIGHT_SERVO:   // P7.4 -> TA1.4
            pwm_value = Angle_To_PWM_Height(angle);
            TIMER_A1->CCR[4] = pwm_value;
            current_height_angle = angle;
            break;

        case TILT_SERVO:     // P7.5 -> TA1.3
            pwm_value = Angle_To_PWM_Tilt(angle);
            TIMER_A1->CCR[2] = pwm_value;
            current_tilt_angle = angle;
            break;

        case GRIPPER_SERVO:  // P7.6 -> TA1.2
            pwm_value = Angle_To_PWM_Gripper(angle);
            TIMER_A1->CCR[3] = pwm_value;
            current_gripper_angle = angle;
            break;
    }
}

void Robot_Arm_Set_Height(uint8_t angle)
{
    Robot_Arm_Set_Servo_Angle(HEIGHT_SERVO, angle);
}

void Robot_Arm_Set_Tilt(uint8_t angle)
{
    Robot_Arm_Set_Servo_Angle(TILT_SERVO, angle);
}

void Robot_Arm_Set_Gripper(uint8_t angle)
{
    Robot_Arm_Set_Servo_Angle(GRIPPER_SERVO, angle);
}

void Robot_Arm_Open_Gripper(void)
{
    Robot_Arm_Set_Gripper(GRIPPER_OPEN);
}

void Robot_Arm_Close_Gripper(void)
{
    Robot_Arm_Set_Gripper(GRIPPER_CLOSED);
}

void Robot_Arm_Move_To_Position(Arm_Position position)
{
    Robot_Arm_Set_Height(position.height_angle);
    Robot_Arm_Set_Tilt(position.tilt_angle);
    Robot_Arm_Set_Gripper(position.gripper_angle);
}

void Robot_Arm_Home(void)
{
    Arm_Position home = ARM_HOME_POSITION;
    Robot_Arm_Move_To_Position(home);
}

void Robot_Arm_Rest(void)
{
    Arm_Position rest = ARM_REST_POSITION;
    Robot_Arm_Move_To_Position(rest);
}

void Robot_Arm_Smooth_Move(Servo_Type servo, uint8_t target_angle, uint16_t step_delay_ms)
{
    uint8_t current_angle;
    int8_t step;
    int i;

    switch(servo) {
        case HEIGHT_SERVO:
            current_angle = current_height_angle;
            break;
        case TILT_SERVO:
            current_angle = current_tilt_angle;
            break;
        case GRIPPER_SERVO:
            current_angle = current_gripper_angle;
            break;
        default:
            return;
    }

    if (target_angle > SERVO_MAX_ANGLE) {
        target_angle = SERVO_MAX_ANGLE;
    }

    if (target_angle > current_angle) {
        step = 1;
    } else if (target_angle < current_angle) {
        step = -1;
    } else {
        return;
    }

    for (i = current_angle; (step > 0) ? (i <= target_angle) : (i >= target_angle); i += step) {
        Robot_Arm_Set_Servo_Angle(servo, i);
        Clock_Delay1ms(step_delay_ms);
    }
}

void Robot_Arm_Pickup_Sequence(void)
{
    Robot_Arm_Open_Gripper();
    Clock_Delay1ms(500);
    Robot_Arm_Set_Tilt(90);
    Clock_Delay1ms(500);
    Robot_Arm_Smooth_Move(HEIGHT_SERVO, 30, 10);
    Clock_Delay1ms(500);
    Robot_Arm_Close_Gripper();
    Clock_Delay1ms(800);
    Robot_Arm_Smooth_Move(HEIGHT_SERVO, 120, 10);
    Clock_Delay1ms(500);
}

void Robot_Arm_Place_Sequence(void)
{
    Robot_Arm_Set_Tilt(90);
    Clock_Delay1ms(500);
    Robot_Arm_Smooth_Move(HEIGHT_SERVO, 40, 10);
    Clock_Delay1ms(500);
    Robot_Arm_Open_Gripper();
    Clock_Delay1ms(800);
    Robot_Arm_Smooth_Move(HEIGHT_SERVO, 120, 10);
    Clock_Delay1ms(500);
}







