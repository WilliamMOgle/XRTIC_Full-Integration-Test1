/*
 * RSLK_Wheel.h
 *
 *  Created on: Feb 27, 2021
 *      Author: wills
 */

#ifndef RSLK_WHEEL_H_
#define RSLK_WHEEL_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <pwm_HAL.h>


#define ENC_TO_RPM_CONST    6
#define MILLI_FACTOR        1000
#define CENTI_FACTOR        100
#define DECI_FACTOR         10
#define UNIT_FACTOR         DECI_FACTOR
#define WHEEL_PWM_FREQ      400
#define STOPPED_RPM_DEFINE  1         //units of RPM
#define NUM_STATE_VARS      2

//#define STOP_THRESHOLD
//#define DEF_CLK             48000000
//const Pin RSLK_right_wheel_pin = {GPIO_PORT_P2, GPIO_PIN6};
//Right wheel pin definitions
#define RSLK_RIGHT_PWM_PORT GPIO_PORT_P2
#define RSLK_RIGHT_PWM_PIN GPIO_PIN6
#define RSLK_RIGHT_SLP_PORT  GPIO_PORT_P3
#define RSLK_RIGHT_SLP_PIN GPIO_PIN6
#define RSLK_RIGHT_DIR_PORT GPIO_PORT_P5
#define RSLK_RIGHT_DIR_PIN  GPIO_PIN5
#define RSLK_RIGHT_ENCB_PORT GPIO_PORT_P5
#define RSLK_RIGHT_ENCB_PIN GPIO_PIN0
#define RSLK_RIGHT_INT_PORT INT_PORT5
#define RSLK_RIGHT_ENCA_PORT GPIO_PORT_P10
#define RSLK_RIGHT_ENCA_PIN GPIO_PIN4
#define RSLK_RIGHT_INT_FLG_REG (&P5IFG)
#define RSLK_RIGHT_INT_FLG_BIT BIT0

//Left wheel pin definitions
#define RSLK_LEFT_PWM_PORT GPIO_PORT_P2
#define RSLK_LEFT_PWM_PIN GPIO_PIN7
#define RSLK_LEFT_SLP_PORT GPIO_PORT_P3
#define RSLK_LEFT_SLP_PIN GPIO_PIN7
#define RSLK_LEFT_DIR_PORT GPIO_PORT_P5
#define RSLK_LEFT_DIR_PIN GPIO_PIN4
#define RSLK_LEFT_ENCB_PORT GPIO_PORT_P5
#define RSLK_LEFT_ENCB_PIN GPIO_PIN2
#define RSLK_LEFT_INT_PORT INT_PORT5
#define RSLK_LEFT_ENCA_PORT GPIO_PORT_P10
#define RSLK_LEFT_ENCA_PIN GPIO_PIN5
#define RSLK_LEFT_INT_FLG_REG (&P5IFG)
#define RSLK_LEFT_INT_FLG_BIT BIT2



typedef enum {
           FORWARD,
           BACKWARD,
           STOPPED
}Wheel_State;

typedef struct {
    uint32_t ticks;
    uint32_t targetTicks;
}Wheel_Rotation_Tracker;

typedef struct {
    //User should set these values
    //uint32_t m_clk;             //Master system clock
    //uint32_t sm_clk;            //Subsystem master clock
    uint32_t sys_clk;
    int32_t des_rpm;           //the desired RPM from the user
    Pin wheel_pwm_pin;          //output PWM for wheel control
    Pin wheel_slp_pin;          //motor sleep pin - active low
    Pin wheel_dir_pin;          //motor direction pin - 0=FWD, 1=BCK
    Pin wheel_enca_pin;
    Pin wheel_encb_pin;
    uint16_t wheel_int_port;
    volatile uint8_t *wheel_int_flg_reg;
    uint8_t int_flg_bit;
    bool enable_compensator;    //enable motor compensator

    //Internal values - user should not set
    uint32_t meas_rpm;          //the measured RPM from the encoders
    //uint32_t deg_count;       //tracks relative degrees rotated
    Wheel_State wheel_state;    //tracks current state of wheel
    uint32_t enc_period;        //number of t32 rollovers between enc pos edges
    uint32_t t32_period_count;  //software timer count for enc period
    uint32_t compensator_count; //determines compensator sampling period
    int32_t p[NUM_STATE_VARS];  //array for the state variables of the compensator
    bool abnormal_wheel_stop;   //flag for when PWM cannot meet desired RPM
    uint32_t t32_stop_count;    //software timer count for max time without wheel rotation
    uint32_t stop_count_thres;  //when t32_stop_count exceeds this value the wheel is defined as stopped
    uint32_t t32_base_count;    //value timer32 is set to continuously count to
    Wheel_Rotation_Tracker wheel_rotation_tracker;      //tracks wheel rotation angle
    PWM_Params pwm_settings;    //PWM control struct

}RSLK_Wheel;

void initRightRSLKWheel(RSLK_Wheel *wheel_data, uint32_t _sys_clk, uint32_t);
void initLeftRSLKWheel(RSLK_Wheel *wheel_data, uint32_t _sys_clk, uint32_t);
void initWheel(RSLK_Wheel *, uint32_t, Pin , Pin , Pin, Pin, Pin, uint16_t, volatile uint8_t*, uint8_t, uint32_t, bool);
uint32_t calcCurrentRPM(RSLK_Wheel *wheel_data);
void initWheelGPIO(RSLK_Wheel *);
void initWheelInterrupts(RSLK_Wheel *);
void enableWheel(RSLK_Wheel *);
void disableWheel(RSLK_Wheel *);
void setWheelDirForward(RSLK_Wheel *);
void setWheelDirBackward(RSLK_Wheel *);
void setWheelDutyCycle(RSLK_Wheel *, double);
void wheelUpdateMove(RSLK_Wheel *);
bool encBInterruptCheck(RSLK_Wheel *);
void updateWheelState(RSLK_Wheel *);
void enableWheelCompensator(RSLK_Wheel *);
void disableWheelCompensator(RSLK_Wheel *);
void clearStateVariables(RSLK_Wheel *);
bool wheelIsInAbnormalWheelStop(RSLK_Wheel *);


#endif /* RSLK_WHEEL_H_ */
