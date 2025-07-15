#ifndef COMM_H
#define COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>

struct udpPacket {
    uint8_t throttle;
    float roll;
    float pitch;
    float yaw;
};

const char* ssid = "donre";

WiFiUDP udp;
unsigned int localPort = 8888;

inline void wifiSetup() {
    WiFi.softAP(ssid);
    Serial.print("WiFi AP started. IP address: ");
    Serial.println(WiFi.softAPIP());

    udp.begin(localPort);
    Serial.print("UDP server started on port: ");
    Serial.println(localPort);
}

inline bool receiveUDPCommand(udpPacket& receivedPacket) {
    int packetSize = udp.parsePacket();

    if (packetSize) {
        if (packetSize >= sizeof(udpPacket)) {
            int len = udp.read((char*)&receivedPacket, sizeof(udpPacket));

            // monitoring command, can be commented in/out as needed
            // if (len > 0) { Serial.print("Throttle: "); Serial.print(receivedPacket.throttle); Serial.print(", Roll: "); Serial.print(receivedPacket.roll, 4); Serial.print(", Pitch: "); Serial.print(receivedPacket.pitch, 4); Serial.print(", Yaw: "); Serial.println(receivedPacket.yaw, 4); }

            return len > 0; // return true if data is successfully read
        } else {
            Serial.println("Received packet too small");
        }
    }
    
    return false; // return false if no packet was received or if it was invalid
}


#endif