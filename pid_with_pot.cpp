/*

	Board: Arduino MEGA

	Instructions: Wait for more than 30 seconds on startup
	because the gyro values need to calibrate.

TO DO:
- only call pid when you have new ypr values
- DOCUMENTATION!!!!?
- change the LED pin to the correct board LED pin 
- change the motors to the correct motor pins
- change the interrupt pin
- understand interrupt pins!
- change the radio pins
- test the radio on the new board (should be the same communication + protocol, 
we're just using a different breakout board)
- calibrate the gyro values
- check irene's messages...she gave advice on how to tune the PID loop
- set the correct pot pin
- change the debug print values to the axis you're tuning
- make sure the signs in the PID loops are correct
- move basically all loop code into the mpuinterrupt loop

Questionable changes:
- just changed base speed and all other speeds to ints (from floats); this could
cause unknown types issues not changed

*/

// the analog pin for the throttle 
int pot_pin_throttle = 4;

#include <Servo.h>;

int s1_pin = 3;
int s2_pin = 5;
int s3_pin = 6;
int s4_pin = 9;

#define INTERRUPT_PIN 2

// set equal to 1.0 when tuning this axis
// set equal to 0.0 when not tuning this axis
float tuning_pitch = 1.0;
float tuning_roll = 0.0;

float kp_r = 0.0;
float ki_r = 0.0;
float kd_r = 0.0;

float kp_p = 0.0;
float ki_p = 0.0;
float kd_p = 0.0;

int speed1 = 0;
int speed2 = 0;
int speed3 = 0;
int speed4 = 0;

float last_theta_r = 0.0;
float last_theta_p = 0.0;

bool intial = true;

float setpoint_r = 0.0;
float setpoint_p = 0.0;

float pid_frequency = 200.0;
float pid_period = 0.0;

unsigned long last_t = 0;

float integral_r = 0.0;
float integral_p = 0.0;

// value to be updated using throttle
int base_speed = 0;

// Change the LED pin  
int led_pin = 0;

//radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CE_PIN 9
#define CSN_PIN 53 // NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

// YOU ARE SHIT AT DOCUMENTATION
const uint8_t x = 0; // 2 element array holding Joystick readings
int t = 1;

//MPU6050 code
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// four rotors
Servo s1, s2, s3, s4;

void setup() {

	pinMode(led_pin, OUTPUT);
	pinMode(INTERRUPT_PIN, INPUT);
	pinMode(pot_pin_throttle, INPUT);
	pid_period = 1 / pid_frequency;

  	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	  Wire.begin();
	  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	  Fastwire::setup(400, true);
	#endif

	// initialize serial communication
  	// (115200 chosen because it is required for Teapot Demo output, but it's
  	// really up to you depending on your project)
  	Serial.begin(115200);

  	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  	// the baud timing being too misaligned with processor ticks. You must use
  	// 38400 or slower in these cases, or use some kind of external separate
  	// crystal solution for the UART timer.

  	// initialize device
  	mpu.initialize();

  	// wait for ready
  	//Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  	while (Serial.available() && Serial.read()); // empty buffer

  	//uncomment next two lines if you need to command when reading starts
  	//while (!Serial.available());                 // wait for data
  	//while (Serial.available() && Serial.read()); // empty buffer again

  	// load and configure the DMP
  	devStatus = mpu.dmpInitialize();

  	// supply your own gyro offsets here, scaled for min sensitivity
  	mpu.setXGyroOffset(220);
  	mpu.setYGyroOffset(76);
  	mpu.setZGyroOffset(-85);
 	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  	// make sure it worked (returns 0 if so)
  	if (devStatus == 0) {
    	// turn on the DMP, now that it's ready
    	mpu.setDMPEnabled(true);

    	// enable Arduino interrupt detection

    	attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    	mpuIntStatus = mpu.getIntStatus();

    	// set our DMP Ready flag so the main loop() function knows it's okay to use it
    	dmpReady = true;

    	// get expected DMP packet size for later comparison
    	packetSize = mpu.dmpGetFIFOPacketSize();
  	}

  	// attach the motor pins to their respective servo objects
  	s1.attach(s1_pin);
	s2.attach(s2_pin);
	s3.attach(s3_pin);
	s4.attach(s4_pin);

	// wait 20 seconds for the gyro values to stabilize
	delay(20000);
	
	// begin radio listening
	radio.begin();
	radio.openReadingPipe(1, pipe);
	radio.startListening();;

	while (true) {
		if (Serial.available()) {

			// get the start command from the computer (the start command is "T0")
			String command =  serial.readString();
			if(command.equals("T0")) {
				break;
			}
		}
	}

	// arm the ESCs
	s1.write(0);
	s2.write(0);
	s3.write(0);
	s4.write(0);
}

void loop() {
	if(intial) {
		last_theta_r = ypr[2];
		last_theta_p = ypr[1];
		last_t = millis();
	}
	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
	// if you are really paranoid you can frequently test in between other
	// stuff to see if mpuInterrupt is true, and if so, "break;" from the
	// while() loop to immediately process the MPU data
	}
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} 
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;


		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		// read the radio values from the UNO controller
		/*if (radio.available()) {
			
			// Dump the payloads until we've gotten everything
			unsigned long message = 0;
		 	bool received = false;

			// Fetch the payload, and see if this was the last one.
			received = radio.read(&message, sizeof(unsigned long));
			Serial.println(message);
			if (message == 201) {
		  		shutoff();
			}

			float p_val = (float)(message / 100000000); 
			message %= 1000000;
			float i_val = (float)(message / 100000);
			message %= 1000;
			float d_val = (float)(message / 100);

			// make the final changes to the tuning gains
			kp_r = p_val * tuning_roll;
			ki_r = i_val * tuning_roll;
			kd_r = d_val * tuning_roll;

			kp_p = p_val * tuning_pitch;
			ki_p = i_val * tuning_pitch;
			kd_p = d_val * tuning_pitch;
		}*/

		//get motor values from the computer. For debugging
		if(Serial.available()) {
		  String command = Serial.readString();

		  // commands for the shutoff
		  if(command.equals("q") || command.equals("x")) {
		  	shutoff();
		  }
		  if(command.charAt(0) == "T") {
			base_speed = command.		  	
		  }
		}	  
	}

  	// add these wherever the PID loop needs to run again
	if(millis() - last_t > pid_period) {
		current_t = millis();	
		pid(ypr[1], ypr[2]);
		last_t = current_t;
	}

}

void pid(float pitch, float roll) {
	/* 
	* inputs:
		* pitch --> float
		* roll  --> float
	* output:
		* none

	Purpose: calculate the correct throttle values for each motor using the tuned PID system
	*/
	base_speed = analogRead(pot_pin_throttle);

	float dt = (float)(current_t - last_t);

	float delta_p_r = kp_r * (roll - setpoint_r);
	integral_r += (roll - setpoint_r) * ki_r * dt;
	float delta_d_r = kd_r * (roll - last_theta_r) / dt;

	float delta_p_p = kp_p * (pitch - setpoint_p);
	integral_p += (pitch - setpoint_p) * ki_p * dt;
	float delta_d_p = kd_p * (pitch - last_theta_p) / dt;

	// fix this...YOUR VARIABLE NAMES ARE SHIT
	float d_theta_p = delta_p_p + integral_p - delta_d_p;
	float d_theta_r = delta_p_r + integral_r - delta_d_r;

	// motor1 is part of axis when lowered, gyro values go positive
	speed1 = base_speed + (int)d_theta_p;
	speed3 = base_speed - (int)d_theta_p;

	// motor2 is part of axis when lowered, gyro values go positive
	speed2 = base_speed + (int)d_theta_r;
	speed4 = base_speed - (int)d_theta_r;

	s1.writeMicroseconds(speed1);
	s2.writeMicroseconds(speed2);
	s3.writeMicroseconds(speed3);
	s4.writeMicroseconds(speed4);

	// debug statements
	Serial.print("Throttle: " + base_speed); 
	Serial.print("\tKp: ");
	Serial.print(kp_r);
	Serial.print("\tKi: ");
	Serial.print(kp_i);
	Serial.print("\tKd: ");
	Serial.println(kp_d);
}

void shutoff() {
	/* This function will shutoff each motor and no longer run the program.
	The quadcopter will have an LED that will flash every half second to 
	signal the shutoff working.
	*/
	s1.write(0);
	s2.write(0);
	s3.write(0);
	s4.write(0);
	while(true) {
		digitalWrite(led_pin, HIGH);
		delay(500);
		digitalWrite(led_pin, LOW);
		delay(500);
	}

}