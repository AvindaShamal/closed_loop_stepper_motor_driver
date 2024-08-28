#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

// Define the I2C address of the AS5600 encoder
#define SLAVE_ADDRESS 0x36  // The I2C address of AS5600 is typically 0x36

// Define the register addresses
#define ANGLE_HIGH_REGISTER 0x0E  // The high byte of the angle register
#define ANGLE_LOW_REGISTER 0x0F   // The low byte of the angle register

float desiredAngle = 200;

#define IN1_PIN PD6
#define IN2_PIN PB3
#define IN3_PIN PB4
#define IN4_PIN PB5

// Function prototypes
void i2c_init();
void i2c_start();
void i2c_stop();
void i2c_write(uint8_t data);
uint8_t i2c_read_ack();
uint8_t i2c_read_nack();
uint16_t readAngle();
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Step sequence for motor control
const uint8_t stepSequence[4][4] = {
	{ 1, 0, 0, 0 },
	{ 0, 0, 1, 0 },
	{ 0, 1, 0, 0 },
	{ 0, 0, 0, 1 }
};
const uint8_t stepSequencerev[4][4] = {
	{ 0, 0, 0, 1 },
	{ 0, 1, 0, 0 },
	{ 0, 0, 1, 0 },
	{ 1, 0, 0, 0 }
};

uint8_t Kp = 300;

// Define the number of steps per revolution for your motor
const int stepsPerRevolution = 200;

void setup() {
	// Set the motor control pins as outputs
	DDRD |= (1 << IN1_PIN);  // Set PD4 as output
	DDRB |= (1 << IN2_PIN) | (1 << IN3_PIN) | (1 << IN4_PIN);  // Set PB3, PB4, PB5 as output
}

void stepMotor(int step) {
	// Set the motor control pins based on the step sequence
	if (stepSequence[step][0]) PORTD |= (1 << IN1_PIN);
	else PORTD &= ~(1 << IN1_PIN);

	if (stepSequence[step][1]) PORTB |= (1 << IN2_PIN);
	else PORTB &= ~(1 << IN2_PIN);

	if (stepSequence[step][2]) PORTB |= (1 << IN3_PIN);
	else PORTB &= ~(1 << IN3_PIN);

	if (stepSequence[step][3]) PORTB |= (1 << IN4_PIN);
	else PORTB &= ~(1 << IN4_PIN);
}

void stepMotorRev(int step) {
	// Set the motor control pins based on the step sequence
	if (stepSequencerev[step][0]) PORTD |= (1 << IN1_PIN);
	else PORTD &= ~(1 << IN1_PIN);

	if (stepSequencerev[step][1]) PORTB |= (1 << IN2_PIN);
	else PORTB &= ~(1 << IN2_PIN);

	if (stepSequencerev[step][2]) PORTB |= (1 << IN3_PIN);
	else PORTB &= ~(1 << IN3_PIN);

	if (stepSequencerev[step][3]) PORTB |= (1 << IN4_PIN);
	else PORTB &= ~(1 << IN4_PIN);
}

int main(void) {
	setup();
	
	// Initialize I2C communication
	i2c_init();

	while (1) {
		uint16_t currentAngle = readAngle();  // Read angle from the encoder

		int16_t stepsToMove = desiredAngle - currentAngle;

		if (stepsToMove > 1) {
			for (int i = 0; i < stepsToMove; i++) {
				stepMotor(i % 4);  // Step through the sequence
				currentAngle = readAngle();  // Read angle from the encoder
				stepsToMove = desiredAngle - currentAngle;
				uint16_t delay = Kp/stepsToMove;
				if (delay < 10) delay = 10;
				while (delay--){
					_delay_ms(1);
				}
			}
		}
		else if(stepsToMove < -1){
			for (int i = 0; i < (-1*stepsToMove) ; i++) {
				stepMotorRev(i % 4);  // Step through the sequence
				currentAngle = readAngle();  // Read angle from the encoder
				stepsToMove = desiredAngle - currentAngle;
				uint16_t delay = Kp/(-stepsToMove);
				if (delay < 10) delay = 10;
				while (delay--){
					_delay_ms(1);
				}
			}
			// Code for moving in the opposite direction could be added here if needed
		}
		
		
	}
}

uint16_t readAngle() {
	uint8_t angleHigh, angleLow;
	uint16_t angle;

	i2c_start();  // Send start condition

	i2c_write(SLAVE_ADDRESS << 1 | 0);  // Send slave address with write bit
	if (!(TWCR & (1 << TWINT))) {
		i2c_stop();     // Error in communication
		return 0xFFFF;  // Return an error code
	}

	i2c_write(ANGLE_HIGH_REGISTER);  // Send register address for high byte
	if (!(TWCR & (1 << TWINT))) {
		i2c_stop();     // Error in communication
		return 0xFFFF;  // Return an error code
	}

	i2c_start();  // Send repeated start condition

	i2c_write(SLAVE_ADDRESS << 1 | 1);  // Send slave address with read bit
	if (!(TWCR & (1 << TWINT))) {
		i2c_stop();     // Error in communication
		return 0xFFFF;  // Return an error code
	}

	angleHigh = i2c_read_ack();  // Read high byte
	angleLow = i2c_read_nack();  // Read low byte

	i2c_stop();  // Send stop condition

	angle = (angleHigh << 8) | angleLow;  // Combine high and low byte to get angle value
	angle = angle & 0x0FFF;  // Mask to get 12-bit angle value

	return map(angle, 0, 4095, 0, 360);  // Map angle to 0-360 degrees
}

