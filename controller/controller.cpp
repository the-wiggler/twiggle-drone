#include <iostream>
#include <string>
#include <sstream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include "controller.hpp"
#include <cmath>
#pragma comment(lib, "ws2_32.lib")

TTF_Font* font = nullptr;

////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////////////////////////////////////////////

int main() {
    /////// SETUP //////////////////////////////////////////////////////////////////////////////
    // SDL
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    std::string fontPath = SDL_GetBasePath();
    fontPath += "font.ttf";
    font = TTF_OpenFont(fontPath.c_str(), 24);
    if (!font) SDL_ShowSimpleMessageBox(0x00000010, "Drone Controller", 
                                        "ERROR: FONT FILE <font.ttf> NOT FOUND", NULL);
    constexpr SDL_Color whiteColor = {255, 255, 255, 255};
    
    // UDP Communication
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in addr = {AF_INET, htons(8888)};
    inet_pton(AF_INET, "192.168.4.1", &addr.sin_addr);

    // drone setup
    uint8_t throttle = motorIdleSpeed; // sets motor speed to idle (this value should match set idle in drone code)
    
    ///////////////////////////////////////////////////////////////////////  /  /  /  /  /  ///////
    /////// MAIN WINDOW LOOP                                        ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////  /  /  /  /  /  ///////
    bool windowRunning = true;
    SDL_Event e;

    uint8_t throttleLevel = motorIdleSpeed;

    while (windowRunning) {
        
        SDL_SetRenderDrawColor(renderer, 37,37,37,255);
        SDL_RenderClear(renderer);
        setControlOutline();

        // check for any SDL window events
        while  (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) {
                windowRunning = false;
            }
        }

        // scans the whole keyboard to check the states of input
        const bool* keyStates = SDL_GetKeyboardState(NULL);
        bool w_pressed = keyStates[SDL_SCANCODE_W]; // if the w key is pressed
        bool s_pressed = keyStates[SDL_SCANCODE_S];
        bool a_pressed = keyStates[SDL_SCANCODE_A];
        bool d_pressed = keyStates[SDL_SCANCODE_D];
        bool space_pressed = keyStates[SDL_SCANCODE_SPACE];

        int8_t control_vector_x = 0; // this is the x component of the direction vector instruction to be sent to the drone
        int8_t control_vector_y = 0; // same as above but for y

        if (w_pressed) {
            control_vector_y = 127; // set a vector component to FORWARD direction
            moveForward();
        }
        if (s_pressed) {
            control_vector_y = -127; // set a vector component to BACKWARD direction
            moveBackward();
        }
        if (a_pressed) { 
            control_vector_x = -127; // set a vector component to LEFT direction
            moveLeft();
        }
        if (d_pressed) {
            control_vector_x = 127; // set a vector component to RIGHT direction
            moveRight();
        }
        if (space_pressed) {
            if (throttleLevel < 251) throttleLevel += 4;
        }        

        /////// SEND UDP DATA //////////////////////////////////////////////////////////////////////
        // udp_data is an array that holds all control data to be sent to the drone. The drone
        // will recognize the bytes in a set format defined below, so it must match on both programs!
        // the first element is what to change the throttle to
        // the second and third element is what vector the drone should aim to change its direction of travel to
        struct udpPacket {
            uint8_t throt;
            int8_t crtl_x;
            int8_t ctrl_y;
        };

        udpPacket packet = { throttleLevel, control_vector_x, control_vector_y };
        int len = sizeof(packet);
        sendto(sock, (const char*)&packet, len, 0, (sockaddr*)&addr, sizeof(addr));

        std::cout << "PACKET CONTENTS (hex): " 
          << "0x" << std::hex << (int)packet.throt 
          << " 0x" << (int)(uint8_t)packet.crtl_x 
          << " 0x" << (int)(uint8_t)packet.ctrl_y 
          << std::dec << std::endl;  // Reset to decimal


        /////// THROTTLE TEXT //////////////////////////////////////////////////////////////////////
        std::stringstream throttleStream;
        throttleStream << "Throttle level: " << static_cast<int>(throttleLevel);
        std::string throttleText = throttleStream.str();
        SDL_Surface* throttleSurface = TTF_RenderText_Blended(font, throttleText.c_str(), throttleText.length(), whiteColor);
        SDL_Texture* throttleTexture = SDL_CreateTextureFromSurface(renderer, throttleSurface);
        SDL_FRect textRect = {50.0f, 50.0f, (float)throttleSurface->w, (float)throttleSurface->h};
        
        SDL_RenderTexture(renderer, throttleTexture, NULL, &textRect);
        SDL_DestroyTexture(throttleTexture);  // Clean up texture
        SDL_DestroySurface(throttleSurface);  // Clean up surface

        /////// TEXT //////////////////////////////////////////////////////////////////////

        /////// TEXT //////////////////////////////////////////////////////////////////////
        
        if (throttleLevel > 50) throttleLevel -= 2;
        SDL_RenderPresent(renderer);
        SDL_Delay(150);
    }

    /////// CLEANUP ////////////////////////////////////////////////////////////////////////////////
    TTF_CloseFont(font);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    closesocket(sock);
    WSACleanup();

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END MAIN
////////////////////////////////////////////////////////////////////////////////////////////////////