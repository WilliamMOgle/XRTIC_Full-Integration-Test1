
#ifndef PWM_HAL_H_
#define PWM_HAL_H_

//INCLUDES
#include <driverlib.h>
//#include <ti/devices/msp432p4xx/inc/msp.h>

//#define DEF_CLK 48000000 //find __SYSTEM_CLOCK in system_msp432p401r.c

#define NUM_PRESCALARS 20
#define NUM_PWM_PINS 16

typedef struct {
    uint16_t portNum; //use gpio.h pin/port definitions
    uint16_t pinNum;
}Pin;

typedef struct {
    uint32_t timer;
    uint8_t ccrx;
    Pin pin;

} PWM_Pin;

typedef struct {
    uint32_t sys_clk;
    double freq; //Hz
    double dutyCycle; // x/100
    Pin output_pin;

    //internal values
    uint8_t prescalar;
    uint16_t ccr0_value;
    int8_t pin_index;
    uint16_t duty_value;
    Timer_A_PWMConfig PWM_config;
} PWM_Params;

Timer_A_PWMConfig PWM_config1;

void setPWMArgs(PWM_Params *pwm_settings, uint32_t sys_clk, double freq, double dutyCycle, uint16_t portNum, uint16_t pinNum);
void setPWM(PWM_Params *pwm_settings);
bool calcPrescalar(PWM_Params *pwm_settings);
bool calcCCR0Value(PWM_Params *pwm_settings);
bool calcDutyValue(PWM_Params *pwm_settings);
bool findPinIndex(PWM_Params *pwm_settings);
void GPIOInit(PWM_Params *pwm_settings);
void generatePWM(PWM_Params *pwm_settings);
bool updateFrequency(double freq, PWM_Params *pwm_settings);
bool updateDutyCycle(double duty_cycle, PWM_Params *pwm_settings);

#endif
