#ifndef COMM_H
#define COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "donre";

WiFiUDP udp;
unsigned int localPort = 8888;

void wifiSetup() {
    WiFi.softAP(ssid);
    udp.begin(localPort);
}

void wifiCommunication() {
    int packetSize = udp.parsePacket();

    if (packetSize) {
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);
        Serial.println(incomingPacket);
        // instead of just printing the packet, I can use the packet data to change throttle info!
        // this will be useful for controls bc I can change the PID setpoint and such
        // im just writing this so I rememebr
    }
}

#endif