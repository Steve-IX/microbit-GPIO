#include "MicroBit.h"

extern NRF52Serial serial;  

#define LOGO_PIN 4  // P1.04


#define TOUCH_INCREMENT_DELAY 1000  //increment (1 second)
#define TOUCH_RESET_DELAY 3000  

#define PWM_FREQUENCY 1000  // Set PWM frequency to 1kHz
#define MAX_BRIGHTNESS 1023 


#define RED_PIN 3    // P1 for red
#define GREEN_PIN 9  // P9 for green
#define BLUE_PIN 10   // P8 for blue

// GPIO positions
#define ROW3_PIN 15   // Row 3 at P0.15
#define COL1_PIN 28   // Column 1 at P0.28
#define COL2_PIN 11   // Column 2 at P0.11
#define COL3_PIN 31   // Column 3 at P0.31
#define COL4_PIN 5    // Column 4 at P0.5
#define COL5_PIN 30   // Column 5 at P0.30

// System Clock
#define SYSTEM_CLOCK_FREQ 64000000  // 64 MHz for nRF52

//buttons
#define BUTTON_A_PIN 14  
#define BUTTON_B_PIN 23  

// Initialize button 
void buttons() {
    
    NRF_P0->PIN_CNF[BUTTON_A_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);

    
    NRF_P0->PIN_CNF[BUTTON_B_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
}

// Initializes GPIO pins for Row 3 and Columns 1-5
void initializeGPIO() {
    NRF_P0->DIRSET = (1 << ROW3_PIN) | (1 << COL1_PIN) | (1 << COL2_PIN) | 
                     (1 << COL3_PIN) | (1 << COL4_PIN) | (1 << COL5_PIN);
    NRF_P1->DIRSET = (1 << COL4_PIN);  

    NRF_P0->OUTSET = (1 << ROW3_PIN);  // Set Row 3 high
}

void delay_us(uint32_t us) {
    SysTick->LOAD = (SYSTEM_CLOCK_FREQ / 1000000) * us - 1;  
    SysTick->VAL = 0;                                        
    SysTick->CTRL = 5;                                       

    //flag is set
    while (!(SysTick->CTRL & 0x10000));

    SysTick->CTRL = 0;  // Disable SysTick
}


void delay_ms(uint32_t ms) {
    // Config
    SysTick->LOAD = (SYSTEM_CLOCK_FREQ / 1000) * ms - 1;  
    SysTick->VAL = 0;                                    
    SysTick->CTRL = 5;                                    

    //flag
    while (!(SysTick->CTRL & 0x10000));

    SysTick->CTRL = 0;  // Disable SysTick
}

// display least 5 significant bits
void displayBinary(uint8_t value) {
    static bool initialized = false;

    //3 and Columns 1-5 on the first call
    if (!initialized) {
        initializeGPIO();
        initialized = true;
    }

    // Clear all relevant columns
    NRF_P0->OUTCLR = (1 << COL1_PIN) | (1 << COL2_PIN) | (1 << COL3_PIN) | (1 << COL5_PIN);
    NRF_P1->OUTCLR = (1 << COL4_PIN);  // Clear Column 4 on NRF_P1

    // Mapping
    if (!(value & 0b00001)) NRF_P0->OUTSET = (1 << COL5_PIN);  // Rightmost (Column 5)
    if (!(value & 0b00010)) NRF_P1->OUTSET = (1 << COL4_PIN);  // Column 4
    if (!(value & 0b00100)) NRF_P0->OUTSET = (1 << COL3_PIN);  // Column 3
    if (!(value & 0b01000)) NRF_P0->OUTSET = (1 << COL2_PIN);  // Column 2
    if (!(value & 0b10000)) NRF_P0->OUTSET = (1 << COL1_PIN);  // Leftmost (Column 1)
}

//delay
void debounceDelay() {
    for (volatile int i = 0; i < 10000; i++); 
}

//subtask2
void countWithButtonsBinary(uint8_t initialValue) {
    serial.printf(" ... Subtask 2 running ... \n");

    buttons();  

    uint8_t value = initialValue;
    displayBinary(value);  // Display initial value

    while (1) {
        // button A(decrement)
        if (!(NRF_P0->IN & (1 << BUTTON_A_PIN))) {  // Active low
            debounceDelay(); 
            if (!(NRF_P0->IN & (1 << BUTTON_A_PIN))) {  
                // Decrement with wrap-around
                value = (value == 0) ? 0b11111 : value - 1;
                displayBinary(value);
                while (!(NRF_P0->IN & (1 << BUTTON_A_PIN)));  
            }
        }

        // button B (increment)
        if (!(NRF_P0->IN & (1 << BUTTON_B_PIN))) {  // Active low
            debounceDelay();  // Debounce delay
            if (!(NRF_P0->IN & (1 << BUTTON_B_PIN))) {  
                // Increment with wrap-around
                value = (value == 0b11111) ? 0 : value + 1;
                displayBinary(value);
                while (!(NRF_P0->IN & (1 << BUTTON_B_PIN)));  
            }
        }
    }
}


void countUpBinary(uint8_t initialValue) {
    serial.printf(" ... Subtask 1 running ... \n");

    uint8_t value = initialValue;

    while (1) {
        displayBinary(value);

        // Increment 
        value = (value + 1) & 0b00011111;

        delay_ms(200);  //delay
    }
}


uint8_t sampleVoltage(void) {
    // Configure SAADC
    NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
                              (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                              (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                              (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
                              (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos);

    // P0
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0;

    // Configure SAADC resolution
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_8bit;
    NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task;

    
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;

    
    int16_t result;
    NRF_SAADC->RESULT.PTR = (uint32_t)&result;
    NRF_SAADC->RESULT.MAXCNT = 1;  

    // Start SAADC 
    NRF_SAADC->TASKS_START = 1;
    NRF_SAADC->TASKS_SAMPLE = 1;

    //converting
    while (!NRF_SAADC->EVENTS_END);
    NRF_SAADC->EVENTS_END = 0;

    NRF_SAADC->TASKS_STOP = 1;
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;

    // 8 bit result (0-255)
    uint8_t scaled_value = (result + 128); // should output -128 to +127
    return scaled_value;
    
}


// Subtask 3
void displayVoltageBinary(void) {
    serial.printf(" ... Subtask 3 running ... \n");

    int previous_voltage = -1;  

    while (1) {
        uint8_t voltage = sampleVoltage();
        uint8_t display_value = voltage >> 3;  //5 most significant bits (0–31)
        
        // Display the voltage level on the LEDs
        displayBinary(display_value);

        // Convert the 8 bit to voltage in millivolts
        int voltage_in_millivolts = (voltage * 3000) / 255;

        // print  voltage
        if (abs(voltage_in_millivolts - previous_voltage) > 10) {
            serial.printf("Measured Voltage: %d mV\n", voltage_in_millivolts);
            previous_voltage = voltage_in_millivolts;
        }

        delay_ms(500);  
    }
}


//SUBTASK 4 //
uint16_t duty_cycle[3] = {0, 0, 0};  

void setPWMDutyCycle(uint16_t red, uint16_t green, uint16_t blue) {
    duty_cycle[0] = red;
    duty_cycle[1] = green;
    duty_cycle[2] = blue;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

void initializePWM() {
    static bool initialized = false;  
    if (initialized) return;

    NRF_PWM0->PSEL.OUT[0] = (RED_PIN << PWM_PSEL_OUT_PIN_Pos) | PWM_PSEL_OUT_CONNECT_Connected;
    NRF_PWM0->PSEL.OUT[1] = (GREEN_PIN << PWM_PSEL_OUT_PIN_Pos) | PWM_PSEL_OUT_CONNECT_Connected;
    NRF_PWM0->PSEL.OUT[2] = (BLUE_PIN << PWM_PSEL_OUT_PIN_Pos) | PWM_PSEL_OUT_CONNECT_Connected;

    NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up; 
    NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;  
    NRF_PWM0->COUNTERTOP = 255;


    NRF_PWM0->SEQ[0].PTR = (uint32_t)&duty_cycle[0];
    NRF_PWM0->SEQ[0].CNT = sizeof(duty_cycle) / sizeof(uint16_t);  
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;
    NRF_PWM0->LOOP = 0;
    NRF_PWM0->DECODER = PWM_DECODER_LOAD_Individual | PWM_DECODER_MODE_RefreshCount;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
    serial.printf("PWM initialized\n");

    initialized = true;  
}

void driveRGB() {
    serial.printf("... Subtask 4 with 1 kHz PWM, breathing effect, and variable resistor control ... \n");

    // Set for RGB control
    NRF_P0->DIRSET = (1 << RED_PIN) | (1 << GREEN_PIN) | (1 << BLUE_PIN);

    int brightness = 0;        //control the brightness level 
    int fadeAmount = 5;        //controls the speed of breathing
    int breathingDelay = 10;   //adjust the delay 

    // Set a 1kHz PWM
    int pwmPeriod = 1;         

    while (1) {
        // Read the variable
        uint8_t pot_value = sampleVoltage();
        float color_ratio = pot_value / 255.0;  //  color ratio

        // duty cycles for each color
        int red_duty = (int)(brightness * color_ratio);
        int green_duty = (int)(brightness * (1 - color_ratio));
        int blue_duty = (int)(brightness * fabs(0.5 - color_ratio) * 2);

        //Red LED
        NRF_P0->OUTSET = (1 << RED_PIN);
        delay_ms(red_duty * pwmPeriod / 100);  // ON
        NRF_P0->OUTCLR = (1 << RED_PIN);
        delay_ms((100 - red_duty) * pwmPeriod / 100);  // OFF 

        // Green LED
        NRF_P0->OUTSET = (1 << GREEN_PIN);
        delay_ms(green_duty * pwmPeriod / 100);
        NRF_P0->OUTCLR = (1 << GREEN_PIN);
        delay_ms((100 - green_duty) * pwmPeriod / 100);

        //Blue LED
        NRF_P0->OUTSET = (1 << BLUE_PIN);
        delay_ms(blue_duty * pwmPeriod / 100);
        NRF_P0->OUTCLR = (1 << BLUE_PIN);
        delay_ms((100 - blue_duty) * pwmPeriod / 100);

        //adjust brightness for breathing effect
        brightness += fadeAmount;
        if (brightness <= 0 || brightness >= 100) {
            fadeAmount = -fadeAmount;  
        }

        // print values
        serial.printf("Red Duty: %d, Green Duty: %d, Blue Duty: %d, Brightness: %d\n", red_duty, green_duty, blue_duty, brightness);

        // delay breathing effect
        delay_ms(breathingDelay);
    }
}

//subtask 5 
void countWithTouchesBinary(uint8_t initialValue) {
    serial.printf(" ... Final Subtask running ... \n");

    uint8_t count = initialValue;
    displayBinary(count);  // initial value

    while (1) {
        
        NRF_P1->PIN_CNF[LOGO_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P1->OUTCLR = (1 << LOGO_PIN);  // Drive the pin to 0V
        delay_ms(1);  // 

        // touch config
        NRF_P1->PIN_CNF[LOGO_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                     (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                     (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);

        // counting
        SysTick->LOAD = SYSTEM_CLOCK_FREQ / 1000000 - 1;  
        SysTick->VAL = 0;  //clear value
        SysTick->CTRL = 5;  

        uint32_t duration = 0;

    
        while (!(NRF_P1->IN & (1 << LOGO_PIN)) && duration < 0xFFFFFFFF) {
            duration++;  // Incrementation
        }

        SysTick->CTRL = 0; 


        if (duration > TOUCH_INCREMENT_DELAY) {  // Short touch for increment
            count = (count + 1) & 0b00011111;  
            displayBinary(count);  //display
            serial.printf("Short Touch! Duration: %d microseconds\n", (unsigned long)duration);
            serial.printf("Incremented Count: %d\n", count);

            
            delay_ms(400); // change speed 

        } else if (duration > TOUCH_RESET_DELAY) {  // Long touch for reset
            count = initialValue;  
            displayBinary(count);  
            serial.printf("Long Touch! Duration: %lu microseconds\n", (unsigned long)duration);
            serial.printf("Count Reset to: %d\n", count);

            
            delay_ms(400); //change speed
        }

        // 
        delay_ms(100);
    }
}

