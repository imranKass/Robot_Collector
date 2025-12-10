/**
 * @file Robot_Arm.h
 *
 * @brief Header file for the Pololu BKT18A Robot Arm driver.
 *
 * This file contains the function definitions for controlling the BKT18A robot arm
 * consisting of three servos:
 *  - Arm Height Servo: Controls vertical position of the arm
 *  - Gripper Tilt Servo: Controls the angle/tilt of the gripper
 *  - Gripper Paddle Servo: Opens and closes the gripper paddles
 *
 * The servos are controlled using PWM signals from Timer A1.
 *
 * Servo connections:
 *  - Arm Height Servo    Signal Pin   <-->  MSP432 LaunchPad Pin P7.4 (TA1.1)
 *  - Gripper Tilt Servo  Signal Pin   <-->  MSP432 LaunchPad Pin P7.5 (TA1.2)
 *  - Gripper Paddle Servo Signal Pin  <-->  MSP432 LaunchPad Pin P7.6 (TA1.3)
 *
 * All servos:
 *  - VCC  <-->  5V External Power Supply
 *  - GND  <-->  Common Ground (MSP432 GND + Power Supply GND)
 *
 * @note Servos typically use 50Hz PWM (20ms period) with pulse widths of 1ms-2ms
 *       1ms pulse = 0 degrees, 1.5ms = 90 degrees, 2ms = 180 degrees
 *
 * @author Your Name
 * @date December 2025
 */

#ifndef INC_ROBOT_ARM_H_
#define INC_ROBOT_ARM_H_

#include <stdint.h>
#include "msp432p401r.h"

/**
 * @brief Servo position limits
 */
#define SERVO_MIN_ANGLE     0      // Minimum servo angle (degrees)
#define SERVO_MAX_ANGLE     60    // Maximum servo angle (degrees)

/**
 * @brief Servo identifiers for BKT18A
 */
typedef enum {
    HEIGHT_SERVO = 0,    // Controls arm height (vertical position)
    TILT_SERVO = 1,      // Controls gripper tilt (angle)
    GRIPPER_SERVO = 2    // Controls gripper paddles (open/close)
} Servo_Type;

/**
 * @brief Gripper states for easy control
 */
typedef enum {
    GRIPPER_OPEN = 0,
    GRIPPER_CLOSED = 180
} Gripper_State;

/**
 * @brief Predefined arm positions
 */
typedef struct {
    uint8_t height_angle;
    uint8_t tilt_angle;
    uint8_t gripper_angle;
} Arm_Position;

// Predefined positions (adjust these based on your arm's mechanical limits)
#define ARM_HOME_POSITION     {90, 90, 90}    // Center position
#define ARM_REST_POSITION     {30, 45, 0}     // Low, tilted down, open
#define ARM_PICKUP_POSITION   {60, 90, 0}     // Medium height, level, open
#define ARM_CARRY_POSITION    {120, 90, 180}  // High, level, closed

/**
 * @brief Initialize the Robot Arm PWM system
 *
 * This function configures Timer A1 to generate 50Hz PWM signals for servo control.
 * All three servos are initialized to their center position (90 degrees).
 *
 * @return None
 */
void Robot_Arm_Init(void);

/**
 * @brief Set the angle of a specific servo
 *
 * @param servo The servo to control (HEIGHT_SERVO, TILT_SERVO, or GRIPPER_SERVO)
 * @param angle Desired angle in degrees (0-180)
 *
 * @return None
 *
 * @note Angles outside the 0-180 range will be clamped to the valid range
 */
void Robot_Arm_Set_Servo_Angle(Servo_Type servo, uint8_t angle);

/**
 * @brief Set the arm height
 *
 * @param angle Desired angle in degrees (0-180)
 *              0° = Lowest position
 *              90° = Mid height
 *              180° = Highest position
 *
 * @return None
 */
void Robot_Arm_Set_Height(uint8_t angle);

/**
 * @brief Set the gripper tilt angle
 *
 * @param angle Desired angle in degrees (0-180)
 *              0° = Tilted down
 *              90° = Level/horizontal
 *              180° = Tilted up
 *
 * @return None
 */
void Robot_Arm_Set_Tilt(uint8_t angle);

/**
 * @brief Set the gripper paddle position
 *
 * @param angle Desired angle in degrees (0-180)
 *              0° = Fully open
 *              180° = Fully closed
 *
 * @return None
 */
void Robot_Arm_Set_Gripper(uint8_t angle);

/**
 * @brief Open the gripper paddles
 *
 * @return None
 */
void Robot_Arm_Open_Gripper(void);

/**
 * @brief Close the gripper paddles
 *
 * @return None
 */
void Robot_Arm_Close_Gripper(void);

/**
 * @brief Move all servos to a predefined position
 *
 * @param position Structure containing desired angles for all servos
 *
 * @return None
 */
void Robot_Arm_Move_To_Position(Arm_Position position);

/**
 * @brief Move the arm to home position (all servos at 90 degrees)
 *
 * @return None
 */
void Robot_Arm_Home(void);

/**
 * @brief Move the arm to rest position (low, tilted, open)
 *
 * @return None
 */
void Robot_Arm_Rest(void);

/**
 * @brief Smoothly move a servo from current position to target angle
 *
 * @param servo The servo to control
 * @param target_angle Desired final angle (0-180)
 * @param step_delay_ms Delay between each step in milliseconds
 *
 * @return None
 *
 * @note This function blocks until the movement is complete
 */
void Robot_Arm_Smooth_Move(Servo_Type servo, uint8_t target_angle, uint16_t step_delay_ms);

/**
 * @brief Execute a pick-up sequence
 *
 * @return None
 *
 * @note This lowers the arm, opens gripper, closes to grab, then lifts
 */
void Robot_Arm_Pickup_Sequence(void);

/**
 * @brief Execute a place-down sequence
 *
 * @return None
 *
 * @note This lowers the arm, opens gripper to release, then lifts
 */
void Robot_Arm_Place_Sequence(void);

#endif /* INC_ROBOT_ARM_H_ */
