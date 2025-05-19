#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "LCD_interface.h"
#include "UART_interface.h"

typedef struct {
	float Kp, Ki, Kd;
	float setpoint, output;
	float error, last_error, integral, derivative;
} PID_Controller;

PID_Controller pid = {
	.Kp = 1.0, .Ki = 0.1, .Kd = 0.05,  // Default reasonable values
	.setpoint = 500.0,                  // Default setpoint in RPM
	.output = 0,
	.error = 0, .last_error = 0,
	.integral = 0, .derivative = 0
};

const uint8_t sample_time = 10; // ms
float last_rpm = 0.0f;          // Stores previous RPM for derivative calculation

// Tachogenerator settings
#define GEN_CONSTANT 0.01f     // Tachogenerator constant (V/RPM) - Adjust based on your generator!
#define MAX_OUTPUT 255         // Maximum PWM output value
#define MIN_OUTPUT 0           // Minimum PWM output value
#define BASE_OUTPUT 100        // Base PWM output when error is zero

// Function prototypes
void init_adc(void);
void init_pwm(void);
void init_ports(void);
uint16_t read_adc(uint8_t channel);
float read_rpm(void);
void update_pid(void);
void display_values(void);

int main(void) {
	// Initialize peripherals
	UART_Vid_Init();
	init_ports();
	init_adc();
	init_pwm();
	LCD_Vid_Init();
	sei();
	UART_Vid_SendString("PID Controller Initialized\r\n");
	
	while(1) {
		// Read PID parameters from potentiometers with appropriate scaling
		pid.Kp = 0.1f + (read_adc(0) / 1023.0f) * 5.0f;     // 0.1-10.0 range
		pid.Ki = 0.001f + (read_adc(1) / 1023.0f) * 2.0f;    // 0.001-0.1 range
		pid.Kd = 0.001f + (read_adc(2) / 1023.0f) * 1.0f;    // 0.001-0.5 range
		pid.setpoint = read_adc(3) / 4.011f;                 // Scale to 0-1000 RPM
		
		update_pid();
		OCR0 = (uint8_t)pid.output;  // Update PWM duty cycle
		
		display_values();
		_delay_ms(sample_time);
	}
}

// Initialize ports
void init_ports(void) {
	DDRA |= 0x00;           // Port A as inputs
	DDRD &= ~(1 << PD0);    // PD0 as input
	
	// Motor control outputs: PB3 (PWM), PB4 and PB5 (direction/status)
	DDRB |= (1 << PB3) | (1 << PB4) | (1 << PB5);
	
	// LCD outputs
	DDRC |= 0xFF;           // All PORTC as outputs
	PORTC = 0x00;
}

// Initialize ADC
void init_adc(void) {
	ADMUX = (1 << REFS0);   // AVcc reference
	ADCSRA = (1 << ADEN) | (7 << ADPS0);  // Enable ADC, prescaler 128
}

// Read ADC value (0-1023)
uint16_t read_adc(uint8_t channel) {
	ADMUX = (1 << REFS0) | (channel & 0x07);  // Set channel
	ADCSRA |= (1 << ADSC);                    // Start conversion
	while(ADCSRA & (1 << ADSC));              // Wait for conversion
	return ADC;
}

// Read RPM from tachogenerator
float read_rpm(void) {
	uint16_t adc_value = read_adc(4);
	float voltage = (adc_value * 5.0f) / 1023.0f;  // Convert ADC to voltage (0-5V)
	float rpm = voltage / GEN_CONSTANT;            // Calculate RPM (V / V/RPM = RPM)
	return rpm;
}

// Initialize PWM on Timer0
void init_pwm(void) {
	DDRB |= (1 << PB3);      // Make sure OC0 (PB3) is output
	// Fast PWM mode, non-inverting, prescaler 8
	TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01);
	OCR0 = 0;               // Start with 0% duty cycle
}

// Update PID controller calculations
void update_pid(void) {
	float rpm = read_rpm();
	
	// Calculate error
	pid.error = pid.setpoint - rpm;
	
	// Proportional term
	float P = pid.Kp * pid.error;
	
	// Integral term with conditional integration and anti-windup
	if(fabs(pid.error) < 100.0f) {  // Only integrate when close to setpoint
		pid.integral += pid.error * (sample_time / 1000.0f);
		// Clamp integral term to prevent windup
		if(pid.integral > 100.0f) pid.integral = 100.0f;
		if(pid.integral < -100.0f) pid.integral = -100.0f;
	}
	else {
		pid.integral *= 0.9f;  // Leaky integrator to prevent windup
	}
	
	float I = pid.Ki * pid.integral;
	
	// Derivative term (on measurement to avoid derivative kick)
	float derivative = (last_rpm - rpm) / (sample_time / 1000.0f);
	float D = pid.Kd * derivative;
	last_rpm = rpm;
	
	// Calculate output
	pid.output = P + I + D;
	
	// Map output to PWM range with base value
	pid.output = BASE_OUTPUT + (pid.output * 0.5f);
	
	// Constrain output to valid PWM range
	if(pid.output > MAX_OUTPUT) {
		pid.output = MAX_OUTPUT;
		PORTB |= (1 << PB4);    // Indicate max output
		PORTB &= ~(1 << PB5);
	}
	else if(pid.output < MIN_OUTPUT) {
		pid.output = MIN_OUTPUT;
		PORTB |= (1 << PB5);    // Indicate min output
		PORTB &= ~(1 << PB4);
	}
	else {
		PORTB &= ~((1 << PB4) | (1 << PB5));  // Clear indicators
	}
}

// Display PID values and RPM on LCD and UART
void display_values(void) {
	float rpm = read_rpm();
	
	// UART Output - Manual float formatting
	char uart_buffer[64];
	char num_buf[10];
	
	// Build UART string manually
	UART_Vid_SendString("Kp:");
	dtostrf(pid.Kp, 5, 2, num_buf);
	UART_Vid_SendString(num_buf);
	
	UART_Vid_SendString(" Ki:");
	dtostrf(pid.Ki, 5, 3, num_buf);
	UART_Vid_SendString(num_buf);
	
	UART_Vid_SendString(" Kd:");
	dtostrf(pid.Kd, 5, 3, num_buf);
	UART_Vid_SendString(num_buf);
	
	UART_Vid_SendString(" RPM:");
	dtostrf(rpm, 6, 1, num_buf);
	UART_Vid_SendString(num_buf);
	
	UART_Vid_SendString(" Set:");
	dtostrf(pid.setpoint, 6, 1, num_buf);
	UART_Vid_SendString(num_buf);
	
	UART_Vid_SendString(" Out:");
	dtostrf(pid.output, 6, 1, num_buf);
	UART_Vid_SendString(num_buf);
	UART_Vid_SendString("\r\n");
	
	// LCD Display (unchanged from previous working version)
	char lcd_line1[17], lcd_line2[17];
	char kp_str[6], ki_str[7], kd_str[7], rpm_str[6];
	
	dtostrf(pid.Kp, 4, 2, kp_str);
	dtostrf(pid.Ki, 5, 3, ki_str);
	dtostrf(pid.Kd, 5, 3, kd_str);
	dtostrf(rpm, 4, 0, rpm_str);
	
	snprintf(lcd_line1, sizeof(lcd_line1), "Kp:%s Ki:%s", kp_str, ki_str);
	snprintf(lcd_line2, sizeof(lcd_line2), "Kd:%s R:%4s", kd_str, rpm_str);
	
	LCD_Vid_Send_Command(0x80);
	LCD_Vid_Send_String(lcd_line1);
	LCD_Vid_Send_Command(0xC0);
	LCD_Vid_Send_String(lcd_line2);
}