/*
 * Zombait Rev 0.0.1
 *
 * Updated: 6/30/2015
 * Author: Matthew F. Borowski 
 * Copyright:  2015 Magurobotics LLC 
 *
 * Revision History
 * 0.0.1 -- MFB Initial Check-In
 */ 

#	define F_CPU 2000000UL //2MHz Unsigned Long

// *** INCLUDES ***
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <inttypes.h>

// *** DEFINITIONS ***
#	define BIT_SET(REGISTER,PIN) ((REGISTER) |= (1<<(PIN)))
#	define BIT_CLEAR(REGISTER,PIN) ((REGISTER) &= ~(1<<(PIN)))
#	define BIT_FLIP(REGISTER,PIN) ((REGISTER) ^= (1<<(PIN)))
#	define BIT_CHECK(REGISTER,PIN) ((REGISTER) & (1<<(PIN)))

// ADC = ((Vbatt/4.48)*1024)/1.1
#define ADC_VALUE_BATTERY_CHARGING_HIGH_4V19		870
#define ADC_VALUE_BATTERY_CHARGING_MID_4V12			856
#define ADC_VALUE_BATTERY_CHARGING_LOW_4V02			835

#define ADC_VALUE_BATTERY_NOLOAD_HIGH_3V25			675
#define ADC_VALUE_BATTERY_NOLOAD_MID_3V12			648
#define ADC_VALUE_BATTERY_NOLOAD_LOW_2V9			602

#define ADC_VALUE_BATTERY_LOAD_HIGH_3V06			635
#define ADC_VALUE_BATTERY_LOAD_MID_2V95				612
#define ADC_VALUE_BATTERY_LOAD_LOW_2V80				581

#define ADC_VALUE_BATTERY_UNDERVOLT_CUTOFF_2V65		550

volatile uint8_t motor_is_running;


//typedef struct LEDs{
	//bool LED1 : 1;
	//bool LED2 : 1;
	//bool LED3 : 1;
	//};
	
// *** FUNCTIONS ***

// Put the Microcontroller to Sleep in Power Down Mode
static inline void _sleep(void){
	BIT_SET(MCUCR, SE); // Sleep Enable
	BIT_SET(MCUCR, SM1); // Idle sleep mode 10 (Power Down)
	BIT_CLEAR(MCUCR, SM0); // Idle sleep mode 10 
}

static inline void _powerSaveMode_ON(void){
	BIT_CLEAR(PORTA, PA7); // Disable Power Supply
	BIT_CLEAR(ADCSRA, ADEN); // ADC Enable bit OFF
}

static inline void _powerSaveMode_OFF(void){
	BIT_SET(PORTA, PA7); // Enable Power Supply
	BIT_SET(ADCSRA, ADEN); // ADC Enable bit ON
}

static inline void _BatteryConnect(void){
	BIT_SET(PORTB, PB1);
}

static inline void _BatteryDisconnect(void){
	BIT_CLEAR(PORTB, PB1);
}

static inline void _motorON(void){
	BIT_SET(PORTB, PB3);
}

static inline void _motorOFF(void){
	BIT_CLEAR(PORTB, PB3);
}

static inline void _ready_H2O(void){
	BIT_CLEAR(PORTA, PA1); // Close Water Sensing Circuit (Active Low)
	BIT_CLEAR(PORTB, PB4); // Open Charge on Electrode A
	BIT_CLEAR(PORTB, PB5); // Open Charge on Electrode B
}

static inline void _ready_Charge(void){
	BIT_SET(PORTA, PA1); // Open Water Sensing Circuit (Active Low)
	BIT_CLEAR(PORTB, PB4); // Open Charge on Electrode A
	BIT_CLEAR(PORTB, PB5); // Open Charge on Electrode B
}

static inline void _ChargeDetect_Enable(void){
	BIT_CLEAR(PORTA, PA3); // Close Charge Detect Circuit (Active Low)
	BIT_CLEAR(PORTB, PB4); // Open Charge on Electrode A
	BIT_CLEAR(PORTB, PB5); // Open Charge on Electrode B
}

static inline void _ChargeDetect_Disable(void){
	BIT_SET(PORTA, PA3); // Open Charge Detect Circuit (Active Low)
	BIT_CLEAR(PORTB, PB4); // Open Charge on Electrode A
	BIT_CLEAR(PORTB, PB5); // Open Charge on Electrode B
}

static inline void _displayLEDs(LED1, LED2, LED3){
	if (LED1 == 1){
	BIT_SET(PORTA, PA4);}
	else{BIT_CLEAR(PORTA, PA4);}
	
	if (LED2 == 1){
	BIT_SET(PORTA, PA5);}
	else{BIT_CLEAR(PORTA, PA5);}
	
	if (LED3 == 1){
	BIT_SET(PORTA, PA6);}
	else{BIT_CLEAR(PORTA, PA6);}
}

static inline void _enterWater(void){
	_motorON();
}

static inline void _exitWater(void){
	_motorOFF();
}

static inline void initADC0(void){
	BIT_CLEAR(PRR, PRADC);  // Disable Power Reduction bit
	BIT_CLEAR(ADCSRB, ADLAR); // ADC results are RIGHT adjusted
	
	BIT_SET(ADMUX, REFS1); // Set Internal 1.1V Bandgap Voltage as Voltage Reference (REFS1 = 1, REFS0 = 0)
	BIT_CLEAR(ADMUX, REFS0); // Set Internal 1.1V Bandgap Voltage as Voltage Reference (REFS1 = 1, REFS0 = 0)
	
	BIT_CLEAR(ADCSRA, ADPS0); // ADC Prescaler = 16
	BIT_CLEAR(ADCSRA, ADPS1); // ADC Prescaler = 16
	BIT_SET(ADCSRA, ADPS2); // ADC Prescaler = 16
}

static inline void initINT0(void) {
	sei(); // Global Interrupt Enable
	
	BIT_CLEAR(MCUCR, ISC00); // Low Level of INT0 triggers Interrupt Request
	BIT_CLEAR(MCUCR, ISC01); // Low Level of INT0 triggers Interrupt Request
	
	BIT_SET(GIMSK, INT0); // General Interrupt Mask Register -- External Interrupt 0 Enable
}

static inline void initINT1(void) {
	sei(); // Global Interrupt Enable
	
	BIT_CLEAR(MCUCR, ISC00); // Low Level of INT1 triggers Interrupt Request
	BIT_CLEAR(MCUCR, ISC01); // Low Level of INT1 triggers Interrupt Request
	
	BIT_SET(GIMSK, INT1); // General Interrupt Mask Register -- External Interrupt 1 Enable
}

static inline void initPCINT8(void) {
	sei(); // Global Interrupt Enable
	
	BIT_SET(GIMSK, PCIE0); // General Interrupt Mask Register -- Pin Change Interrupt Enable 0
	BIT_SET(PCMSK1, PCINT8); // Enable PCINT8
	BIT_CLEAR(PCMSK1, PCINT9); // Disable PCINT9
	BIT_CLEAR(PCMSK1, PCINT10); // Disable PCINT10
	BIT_CLEAR(PCMSK1, PCINT11); // Disable PCINT11
	
	BIT_CLEAR(GIMSK, PCIE1); // General Interrupt Mask Register -- Pin Change Interrupt Disable 1
	BIT_CLEAR(PCMSK0, PCINT0); // Disable PCINT0
	BIT_CLEAR(PCMSK0, PCINT1); // Disable PCINT1
	BIT_CLEAR(PCMSK0, PCINT2); // Disable PCINT2
	BIT_CLEAR(PCMSK0, PCINT3); // Disable PCINT3
	BIT_CLEAR(PCMSK0, PCINT4); // Disable PCINT4
	BIT_CLEAR(PCMSK0, PCINT5); // Disable PCINT5
	BIT_CLEAR(PCMSK0, PCINT6); // Disable PCINT6
	BIT_CLEAR(PCMSK0, PCINT7); // Disable PCINT7
	BIT_CLEAR(PCMSK1, PCINT12); // Disable PCINT12
	BIT_CLEAR(PCMSK1, PCINT13); // Disable PCINT13
	BIT_CLEAR(PCMSK1, PCINT14); // Disable PCINT14
	BIT_CLEAR(PCMSK1, PCINT15); // Disable PCINT15
}

	// ********** Interrupt Vectors ********** //
ISR(INT0_vect){ // What to do when Power on Electrode A
	
	_motorOFF();
	
	_displayLEDs(1,0,1);
	_delay_ms(1000);
	_displayLEDs(0,0,0);
	_delay_ms(1000);
	_displayLEDs(1,0,1);
	_delay_ms(5000);
}

ISR(INT1_vect){ // What to do when Power on Electrode B
		
		_motorOFF();
		
		_displayLEDs(0,1,0);
		_delay_ms(1000);
		_displayLEDs(0,0,0);
		_delay_ms(1000);
		_displayLEDs(0,1,0);
		_delay_ms(5000);
}

ISR(PCINT_vect){
		if (bit_is_set(PINB, PB0)){
			_delay_ms(500);
			if  (bit_is_set(PINB, PB0)){
				_enterWater();
			}
		}
		else{
			_exitWater();
		}
	
}


int main(void){
	// ********** Initialization ********** //
	
	/*
 * Pin Configuration
 * -----------------
 * PA0 -- ADC0 (V_battery)
 * PA1 -- Output (Ready_H2O)
 * PA2 -- INT1 (PowerIn_ElectrodeB)
 * PA3 -- OUTPUT Charge Enable <--- Active Low
 * PA4 -- OUTPUT (LED_1_EN)
 * PA5 -- OUTPUT (LED_2_EN)
 * PA6 -- OUTPUT (LED_3_EN)
 * PA7 -- OUTPUT (PS_EN)
 * PB0 -- PCINT8 (In_H2O)
 * PB1 -- OUTPUT (Battery_Connected_To_Load)
 * PB2 -- NOT USED
 * PB3 -- OUTPUT (Motor_EN)
 * PB4 -- OUTPUT (Charge_ElectrodeA)
 * PB5 -- OUTPUT (Charge_ElectrodeB)
 * PB6 -- INT0 (PowerIn_ElectrodeA)
 * PB7 -- INPUT (In Water) + (RESET) 
 */
	
	// Set I/O configuration
	DDRA = 0b11111010;	// PORT A I/O Config -- 1 = OUTPUT, 0 = INPUT
	PORTA = (0b10000010);  // 1 on Output enables output, 1 on Input enables Pull-up
	DDRB = 0b10111110;	// PORT B I/O Config -- 1 = OUTPUT, 0 = INPUT
	PORTB = (0b10000000); // 1 on Output enables output, 1 on Input enables Pull-up
	
	initADC0(); // Initialize the ADC converter
	initINT0(); // Initialize Interrupt 0 for Charge on Electrode A
	initINT1(); // Initialize Interrupt 1 for Charge on Electrode B
	initPCINT8(); // Initialize PC Interrupt 8 for Water Detection Circuit
	_BatteryConnect(); // Battery Disconnect is off
	
	_displayLEDs(0,0,0); // Turn all LEDs off to start with
	

	// ********** Main Loop ********** //
	while(1){
		//_sleep();
		_powerSaveMode_OFF(); // Should be ON (Disables Power Supply + ADC)
		
			_ChargeDetect_Enable(); // Disable Charge Detection Circuit
			_ready_H2O(); // Device is ready for water if the UnderVoltageInterrupt is closed
		
	}
}