#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <cmath>

#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "PID.h"
#include "debug.h"
#include "communication.h"

SemaphoreHandle_t sensorMutex;
SemaphoreHandle_t controlPacketMutex;

// buffers for control packets
udpPacket control_packet_buffer[2];
volatile int active_buffer = 0;
volatile int write_buffer = 1;

// packet timeout
unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT_MS = 2000; // 2 second timeout
bool packetTimeout = false;

////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void core0Process(void *parameter);

void setup() {
	Serial.begin(115200);
	delay(3000); // small delay to allow serial monitor to set up

	// initialize mutexes for core 0 & 1 shared variables
	sensorMutex 		= xSemaphoreCreateMutex();
	controlPacketMutex 	= xSemaphoreCreateMutex();

	// sets initial default value to control packet buffers
	control_packet_buffer[0] = {0, 0, 0, 0};
	control_packet_buffer[1] = {0, 0, 0, 0};

	lastPacketTime = millis();
	packetTimeout = false;

	wifiSetup();

	/////// MOTOR INITIALIZATION ///////////////////////////////////////////////////////////////////
	// front left motor
	ledcSetup(MOTOR_FL, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_FL_PIN, MOTOR_FL);

	//front right motor
	ledcSetup(MOTOR_FR, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_FR_PIN, MOTOR_FR);

	// rear left motor
	ledcSetup(MOTOR_RL, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_RL_PIN, MOTOR_RL);

	// rear right motor
	ledcSetup(MOTOR_RR, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttachPin(MOTOR_RR_PIN, MOTOR_RR);

	// set all motor speeds to 0 at startup
	ledcWrite(MOTOR_FL, 0);
	ledcWrite(MOTOR_FR, 0);
	ledcWrite(MOTOR_RL, 0);
	ledcWrite(MOTOR_RR, 0);

	Serial.println("Motors initialized.");
	////////////////////////////////////////////////////////////////////////////////////////////////

	/////// SENSOR INITIALIZATION //////////////////////////////////////////////////////////////////
	for (int i = 0; i < 11; i++) {
		if (icm.begin_I2C()) {
			Serial.println("ICM20948 found!");
			break;
		}
		Serial.println("Failed to connect to ICM20948. Retrying...");
		delay(1000);
	}
	// for some reason it sometimes doesnt connect on the first try...idk why???

	icm.setAccelRateDivisor(0); 	// maximum accelerometer rate
	icm.setGyroRateDivisor(0); 		// maximum gyroscope rate

	Serial.println("Sensors initialized.");
	////////////////////////////////////////////////////////////////////////////////////////////////

	/////// CORE TASK SETUP ////////////////////////////////////////////////////////////////////////
	xTaskCreatePinnedToCore(
		core0Process,
		"core0Process",
		20000,
		NULL,
		2,
		NULL,
		0
	);
	////////////////////////////////////////////////////////////////////////////////////////////////
	
	// wait 3 seconds
	Serial.println("Startup...");
	delay(1000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
// CORE 1 LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SYSTEM_FAILURE = false;
constexpr unsigned long LOOP_INTERVAL = 5000; // 5ms is a 200 Hz delay for the PID system

// this is a variable that core 0 writes to when it recieves a packet
// core 1 uses setpoint data from this packet to make adjustments to the drone orientation via the
// PID functions
udpPacket control_packet;

void loop() {
	if (millis() - lastPacketTime > PACKET_TIMEOUT_MS && lastPacketTime != 0) {
		if(!packetTimeout) {
			setAllMotorSpeed(0);
			updateMotorSpeed();

			Serial.println("PACKET TIMEOUT - MOTORS KILLED");
			packetTimeout = true;
			monitorMotorSpeeds();
		}
		return;
	}

	// restores motor control if a packet has been received 
	if (packetTimeout) packetTimeout = false;

	// this collects info from control_packet so its memory safe
	udpPacket local_control_packet;

	// read of control_packet
	if (xSemaphoreTake(controlPacketMutex, 0)) {	// 0 means dont wait if its taken
		local_control_packet = control_packet_buffer[active_buffer];
		xSemaphoreGive(controlPacketMutex); // release packet mutex
	}
	// basically if the mutex is in use it uses previous packet data until it can read more

	//Serial.print("\rRoll Setpoint: "); Serial.print(local_control_packet.roll); Serial.print(" | ");
	//Serial.print("Pitch Setpoint: "); Serial.print(local_control_packet.pitch); Serial.print(" | ");
	//Serial.print("Throttle: "); Serial.print(local_control_packet.throttle);

	unsigned long  startTime = micros();

	// check if a failure has been detected
	// if (millis() > 15000) emergencyStop(); // timer to run the program until a cutoff time
	if (SYSTEM_FAILURE) return;

	float rollOutput, pitchOutput;

	// calculate PID outputs if the sensor data is safe to access (not being used)
	if (xSemaphoreTake(sensorMutex, 10 / portTICK_PERIOD_MS)) {
		rollOutput 	= calculatePID(ROLL_PID, rollErrors, local_control_packet.roll, orientations[0]);
		pitchOutput = calculatePID(PITCH_PID, pitchErrors, local_control_packet.pitch, orientations[1]);
		xSemaphoreGive(sensorMutex);
	}
	else return; // in case core 0 was using our data :(

	// placeholder since im lazy and havent implemented yaw stuff yet
	float yawOutput = 0;

	// changes the state of the motor speed arrays to PID corrected values
	updateMotorsFromPID(rollOutput, pitchOutput, yawOutput, local_control_packet.throttle);

	// sends the new motor speeds to the PWM hardware
	updateMotorSpeed();

	//monitorRollPitchPID(rollOutput, pitchOutput);
	monitorMotorSpeeds();

	// maintains the 200 Hz clock speed
	unsigned long elapsedTime = micros() - startTime;
	if (elapsedTime < LOOP_INTERVAL) delayMicroseconds(LOOP_INTERVAL - elapsedTime); 
	// apparently delayMicroseconds is a thing!
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END CORE 1
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// CORE 0 LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void core0Process(void *parameter) {
	while(true) {
		if (!SYSTEM_FAILURE) {
			udpPacket received_packet;

			// only updates control_packet if a new packet was received
			if(receiveUDPCommand(received_packet)) {
				control_packet_buffer[write_buffer] = received_packet;

				// responsible for keeping track of connection status
				lastPacketTime = millis();
				packetTimeout = false;

				// if available, writes newly recieved packet to global control_packet variable
				if(xSemaphoreTake(controlPacketMutex, 3 / portTICK_PERIOD_MS)) {
					int temp = active_buffer;
					active_buffer = write_buffer;
					write_buffer = temp;
					xSemaphoreGive(controlPacketMutex);
				}
				// if the mutex cant be written to, the new packet will be used next time!
			}

			if (xSemaphoreTake(sensorMutex, portMAX_DELAY)) {
				readICM();
				calculateOrientation();
				xSemaphoreGive(sensorMutex);
			}
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END CORE 0
////////////////////////////////////////////////////////////////////////////////////////////////////