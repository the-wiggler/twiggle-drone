#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "PID.h"
#include "debug.h"
#include "communication.h"

SemaphoreHandle_t controlPacketMutex;
SemaphoreHandle_t pidPacketMutex;

// buffers for control packets
udpPacket controlPacketBuffer[2];
volatile int activeBuffer = 0;
volatile int writeBuffer = 1;

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

	// potential fix to motors running a small amount when the battery gives initial power
	digitalWrite(MOTOR_FL_PIN, LOW);
	digitalWrite(MOTOR_FR_PIN, LOW);
	digitalWrite(MOTOR_RL_PIN, LOW);
	digitalWrite(MOTOR_RR_PIN, LOW);

	delay(3000); // small delay to allow serial monitor to set up

	// initialize mutexes for core 0 & 1 shared variables
	controlPacketMutex 	= xSemaphoreCreateMutex();
	pidPacketMutex 		= xSemaphoreCreateMutex();

	// sets initial default value to control packet buffers
	controlPacketBuffer[0] = {0, 0, 0, 0};
	controlPacketBuffer[1] = {0, 0, 0, 0};

	lastPacketTime = millis();
	packetTimeout = false;

	wifiSetup();

	initializeMotors();
	initializeSensors();

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
	
	// wait
	Serial.println("Startup...");
	delay(1000);

	initializePID();
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
udpPacket pid_packet;

void loop() {

	// checks if a packet has been received from the controller recently, if not, it kills motors
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

	udpPacket localControlPacket;
	// read of control_packet
	if (xSemaphoreTake(controlPacketMutex, 0)) {	// 0 means dont wait if its taken
		localControlPacket = controlPacketBuffer[activeBuffer];
		xSemaphoreGive(controlPacketMutex); // release packet mutex
	}
	
	// skips motor control if a system failure is detected
	if (SYSTEM_FAILURE) return;

	unsigned long  startTime = micros();	

	// reads the gyroscope data and writes it to sensor arrays
	readICM();

	// calculates the orientation of the drone (in radians) based off of stored sensor data
	calculateOrientation();

	// calculate PID outputs if the sensor data is safe to access (not being used)
	PIDControl(localControlPacket.roll, localControlPacket.pitch, localControlPacket.yaw, 
				localControlPacket.throttle);

	// sends the new motor speeds to the PWM hardware
	updateMotorSpeed();

	// debug stuff
	//monitorRollPitchPID(rollOutput, pitchOutput, yawOutput);
	monitorMotorSpeeds();
	//logData(rollOutput, pitchOutput);

	// maintains the 200 Hz clock speed
	unsigned long elapsedTime = micros() - startTime;
	if (elapsedTime < LOOP_INTERVAL) delayMicroseconds(LOOP_INTERVAL - elapsedTime); 
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
			udpPacket receivedPacket;

			// only updates control_packet if a new packet was received
			if (receiveUDPCommand(receivedPacket) && receivedPacket.identifier == 'c') {
				controlPacketBuffer[writeBuffer] = receivedPacket;

				// responsible for keeping track of connection status
				lastPacketTime = millis();
				packetTimeout = false;

				// if available, writes newly recieved packet to global control_packet variable
				if (xSemaphoreTake(controlPacketMutex, 3 / portTICK_PERIOD_MS)) {
					int temp = activeBuffer;
					activeBuffer = writeBuffer;
					writeBuffer = temp;
					xSemaphoreGive(controlPacketMutex);
				}
				// if the mutex cant be written to, the new packet will be used next time!
			}
			else if (receivedPacket.identifier == 'p') {
				if(xSemaphoreTake(pidPacketMutex, 3 / portTICK_PERIOD_MS)) {
					pid_packet = receivedPacket;
					xSemaphoreGive(pidPacketMutex);
				}
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS); 
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END CORE 0
////////////////////////////////////////////////////////////////////////////////////////////////////