#pragma once
#include "driver/gpio.h"

// Assign GPIO pins and configure options in this file


//===========================
//===== Input GPIO pins =====
//===========================
// all inputs are active LOW (switches wired to pull input to GND)
#define CONFIG_ENCODER_A_GPIO     GPIO_NUM_36  // board pullup desoldered for encoder module to work
#define CONFIG_ENCODER_B_GPIO     GPIO_NUM_39  // board pullup desoldered for encoder module to work
#define CONFIG_ENCODER_SW_GPIO    GPIO_NUM_34

#define CONFIG_POS_SWITCH_GPIO    GPIO_NUM_35
#define CONFIG_BUTTON_1_GPIO      GPIO_NUM_32
#define CONFIG_BUTTON_2_GPIO      GPIO_NUM_33



//============================
//===== Output GPIO pins =====
//============================
// all outputs are active high (device turns on when output high)
#define CONFIG_BUZZER_GPIO          GPIO_NUM_17
#define CONFIG_LED1_GPIO            GPIO_NUM_26
#define CONFIG_LED2_GPIO            GPIO_NUM_25

#define CONFIG_MOS_VALVE_GPIO       GPIO_NUM_5
#define CONFIG_MOS_RESERVE_GPIO     GPIO_NUM_18
#define CONFIG_OPEN_DRAIN_RESERVE_GPIO GPIO_NUM_14

#define CONFIG_RELAY_MOTOR_GPIO     GPIO_NUM_27
#define CONFIG_RELAY_230V_GPIO      GPIO_NUM_2



 
//=======================================
//==== HX711 24 bit ADC (load-cell) =====
//=======================================
#define CONFIG_HX711_DT_GPIO        GPIO_NUM_19 // re-routed using jumper wire on the back
//#define CONFIG_HX711_DT_GPIO        GPIO_NUM_12 //according to pcb layout, but causes crash on nvs write (strapping pin)
#define CONFIG_HX711_SCK_GPIO       GPIO_NUM_13

#define CONFIG_HX711_POLL_INTERVAL_MS 400 //AVG vs INTERVAL: consider one readout takes ~90ms 
#define CONFIG_HX711_AVG_SAMPLE_COUNT 4




//===============================
//====== I2C OLED Display =======
//===============================
#define CONFIG_DISPLAY_SDA_GPIO     GPIO_NUM_21
#define CONFIG_DISPLAY_SCL_GPIO     GPIO_NUM_22




//=======================
//======== Servo ========
//=======================
#define CONFIG_SERVO_PWM_GPIO       GPIO_NUM_4  // Servo PWM Signal: directly connected to terminal (no pulldown present)
#define CONFIG_SERVO_ENABLE_GPIO    GPIO_NUM_16  // P-MOSFET Enable supply for Servo (outputs 12V when HIGH)