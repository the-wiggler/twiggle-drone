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

void recieveUDPCommand() {
    int packetSize = udp.parsePacket();

    if (packetSize) {
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);

        if (len > 0) {
            Serial.print("Received Packet: ");
            for (int i = 0; i < len; i++) {
                Serial.print(incomingPacket[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
}

#endif