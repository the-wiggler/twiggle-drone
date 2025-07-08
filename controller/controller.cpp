#include <iostream>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

// im going to be honest I dont really understand how this works yet so ill just leave it at this lol

int main() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in addr = {AF_INET, htons(8888)};
    inet_pton(AF_INET, "192.168.4.1", &addr.sin_addr);

    std::string cmd;
    while (std::cout << "> ", std::getline(std::cin, cmd) && cmd != "quit") {
        if (!cmd.empty()) {
            sendto(sock, cmd.c_str(), cmd.length(), 0, (sockaddr*)&addr, sizeof(addr));
            std::cout << "Sent: " << cmd << std::endl;
        }
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}