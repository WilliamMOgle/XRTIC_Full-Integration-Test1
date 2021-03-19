/*
 * RSLK_Wheel.c
 *
 *  Created on: Feb 27, 2021
 *      Author: wills
 */


#include <RSLK_Wheel.h>


//const Pin RSLK_right_wheel_pin = {GPIO_PORT_P2, GPIO_PIN6};
//const Pin RSLK_left_wheel_pin = {GPIO_PORT_P2, GPIO_PIN7};

void initRightRSLKWheel(RSLK_Wheel *wheel_data,uint32_t _sys_clk, uint32_t _t32_base_cnt)
{

    Pin output_pin = {RSLK_RIGHT_PWM_PORT, RSLK_RIGHT_PWM_PIN};
    Pin dir_pin = {RSLK_RIGHT_DIR_PORT, RSLK_RIGHT_DIR_PIN};
    Pin slp_pin = {RSLK_RIGHT_SLP_PORT, RSLK_RIGHT_SLP_PIN};
    Pin enca_pin = {RSLK_RIGHT_ENCA_PORT, RSLK_RIGHT_ENCA_PIN};
    Pin encb_pin = {RSLK_RIGHT_ENCB_PORT, RSLK_RIGHT_ENCB_PIN};
    uint16_t int_port = RSLK_RIGHT_INT_PORT;
    uint8_t *int_flg_reg = RSLK_RIGHT_INT_FLG_REG;
    uint8_t int_flg_bit = RSLK_RIGHT_INT_FLG_BIT;

    initWheel(wheel_data, _sys_clk,   output_pin,
                                      dir_pin,
                                      slp_pin,
                                      enca_pin,
                                      encb_pin,
                                      int_port,
                                      int_flg_reg,
                                      int_flg_bit,
                                      _t32_base_cnt);


}

void initLeftRSLKWheel(RSLK_Wheel *wheel_data,uint32_t _sys_clk, uint32_t _t32_base_cnt)
{
    Pin output_pin = {RSLK_LEFT_PWM_PORT, RSLK_LEFT_PWM_PIN};
    Pin dir_pin = {RSLK_LEFT_DIR_PORT, RSLK_LEFT_DIR_PIN};
    Pin slp_pin = {RSLK_LEFT_SLP_PORT, RSLK_LEFT_SLP_PIN};
    Pin enca_pin = {RSLK_LEFT_ENCA_PORT, RSLK_LEFT_ENCA_PIN};
    Pin encb_pin = {RSLK_LEFT_ENCB_PORT, RSLK_LEFT_ENCB_PIN};
    uint16_t int_port = RSLK_LEFT_INT_PORT;
    uint8_t *int_flg_reg = RSLK_LEFT_INT_FLG_REG;
    uint8_t int_flg_bit = RSLK_LEFT_INT_FLG_BIT;

    initWheel(wheel_data, _sys_clk,   output_pin,
                                      dir_pin,
                                      slp_pin,
                                      enca_pin,
                                      encb_pin,
                                      int_port,
                                      int_flg_reg,
                                      int_flg_bit,
                                      _t32_base_cnt);


}

void initWheel(RSLK_Wheel *wheel_data, uint32_t _sys_clk, Pin output_pin, Pin dir_pin, Pin slp_pin,
               Pin enca_pin, Pin encb_pin, uint16_t int_port, volatile uint8_t *int_flg_reg, uint8_t int_flg_bit,
               uint32_t _t32_base_cnt)
{
    //User determined data
    wheel_data->sys_clk = _sys_clk;
    wheel_data->des_rpm = 0;
    wheel_data->wheel_pwm_pin.pinNum = output_pin.pinNum;
    wheel_data->wheel_pwm_pin.portNum = output_pin.portNum;
    wheel_data->wheel_slp_pin.pinNum = slp_pin.pinNum;
    wheel_data->wheel_slp_pin.portNum = slp_pin.portNum;
    wheel_data->wheel_dir_pin.pinNum = dir_pin.pinNum;
    wheel_data->wheel_dir_pin.portNum = dir_pin.portNum;
    wheel_data->wheel_enca_pin.pinNum = enca_pin.pinNum;
    wheel_data->wheel_enca_pin.portNum = enca_pin.portNum;
    wheel_data->wheel_encb_pin.pinNum = encb_pin.pinNum;
    wheel_data->wheel_encb_pin.portNum = encb_pin.portNum;
    wheel_data->wheel_int_port = int_port;
    wheel_data->wheel_int_flg_reg = int_flg_reg;
    wheel_data->int_flg_bit = int_flg_bit;
    wheel_data->t32_base_count = _t32_base_cnt;

    //Internal Data
    wheel_data->enc_period = wheel_data->sys_clk;
    wheel_data->wheel_state = STOPPED;
    wheel_data->meas_rpm = 0;
    wheel_data->t32_period_count = 0;
    wheel_data->t32_stop_count = 0;
    wheel_data->wheel_rotation_tracker.ticks = 0;
    wheel_data->wheel_rotation_tracker.targetTicks = 0;
    wheel_data->stop_count_thres =  wheel_data->sys_clk / ENC_TO_RPM_CONST / STOPPED_RPM_DEFINE / wheel_data->t32_base_count ;

    wheel_data->pwm_settings.sys_clk = _sys_clk;
    wheel_data->pwm_settings.freq = WHEEL_PWM_FREQ;
    wheel_data->pwm_settings.dutyCycle = 0;
    wheel_data->pwm_settings.output_pin.pinNum = wheel_data->wheel_pwm_pin.pinNum;
    wheel_data->pwm_settings.output_pin.portNum = wheel_data->wheel_pwm_pin.portNum;

    initWheelGPIO(wheel_data);
    initWheelInterrupts(wheel_data);
    setPWM(&wheel_data->pwm_settings);

}

void enableWheel(RSLK_Wheel *wheel_data)
{
    GPIO_setOutputHighOnPin(wheel_data->wheel_slp_pin.portNum, wheel_data->wheel_slp_pin.pinNum);
}

void disableWheel(RSLK_Wheel *wheel_data)
{
    GPIO_setOutputLowOnPin(wheel_data->wheel_slp_pin.portNum, wheel_data->wheel_slp_pin.pinNum);
}

void setWheelDirForward(RSLK_Wheel *wheel_data)
{
    GPIO_setOutputLowOnPin(wheel_data->wheel_dir_pin.portNum, wheel_data->wheel_dir_pin.pinNum);
}

void setWheelDirBackward(RSLK_Wheel *wheel_data)
{
    GPIO_setOutputHighOnPin(wheel_data->wheel_dir_pin.portNum, wheel_data->wheel_dir_pin.pinNum);
}

void initWheelGPIO(RSLK_Wheel *wheel_data)
{
    GPIO_setAsInputPin(wheel_data->wheel_enca_pin.portNum, wheel_data->wheel_enca_pin.pinNum);  //Encoder inputs from motor
    GPIO_setAsInputPin(wheel_data->wheel_encb_pin.portNum, wheel_data->wheel_encb_pin.pinNum);
    GPIO_setAsOutputPin(wheel_data->wheel_slp_pin.portNum, wheel_data->wheel_slp_pin.pinNum);   //right motor sleep pin -active low
    GPIO_setAsOutputPin(wheel_data->wheel_dir_pin.portNum, wheel_data->wheel_dir_pin.pinNum);   //Right Direction pin. 0=FWD, 1=BCK
}

void initWheelInterrupts(RSLK_Wheel *wheel_data)
{
    GPIO_interruptEdgeSelect(wheel_data->wheel_encb_pin.portNum, wheel_data->wheel_encb_pin.pinNum, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(wheel_data->wheel_encb_pin.portNum, wheel_data->wheel_encb_pin.pinNum);
    GPIO_enableInterrupt(wheel_data->wheel_encb_pin.portNum, wheel_data->wheel_encb_pin.pinNum);
    Interrupt_enableInterrupt(wheel_data->wheel_int_port);
    P5IFG = 0;
}

bool encBInterruptCheck(RSLK_Wheel *wheel_data)
{
    return (*(wheel_data->wheel_int_flg_reg) & wheel_data->int_flg_bit);
}

void setWheelDutyCycle(RSLK_Wheel *wheel_data, double duty_cycle)
{
    wheel_data->pwm_settings.dutyCycle = duty_cycle;
    //setPWM(&wheel_data->pwm_settings);
    updateDutyCycle(duty_cycle, &wheel_data->pwm_settings);
}

void wheelUpdateMove(RSLK_Wheel *wheel_data)
{
    generatePWM(&wheel_data->pwm_settings);
}

void updateWheelState(RSLK_Wheel *wheel_data)
{
    if(wheel_data->t32_stop_count  >= wheel_data->stop_count_thres)
    {
        wheel_data->wheel_state = STOPPED;
        wheel_data->enc_period = wheel_data->sys_clk;
    }
}


void setFowardWheelSpeedAsPercent(RSLK_Wheel *wheel_data, double percent)
{

}

uint32_t calcCurrentRPM(RSLK_Wheel *wheel_data)
{
    uint32_t rpm = wheel_data->sys_clk / wheel_data->enc_period * UNIT_FACTOR / ENC_TO_RPM_CONST / wheel_data->t32_base_count;
    wheel_data->meas_rpm = rpm;
    return  wheel_data->meas_rpm;
}

