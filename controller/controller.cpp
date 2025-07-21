#include <iostream>
#include <string>
#include <sstream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include "controller.hpp"
#include <cmath>
#include <chrono>
#include <thread>
#pragma comment(lib, "ws2_32.lib")

TTF_Font* font = nullptr;

// hi there if you're reading this :)
// im sorry because this controller program is very poorly formatted im just kind of throwing it together
// good luck!

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
    
    // Clickable parameter boxes
    float proportional = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;

    ///////////////////////////////////////////////////////////////////////  /  /  /  /  /  ///////
    /////// MAIN WINDOW LOOP                                        ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////  /  /  /  /  /  ///////
    bool windowRunning = true;
    bool motors_killed = false;
    bool pid_tuning = false; // set to true if you want to modify PID data via UDP (dangerous)
    SDL_Event e;

    uint8_t throttleLevel = motorIdleSpeed;

    while (windowRunning) {
        bool box_clicked = false;
        
        SDL_SetRenderDrawColor(renderer, 37,37,37,255);
        SDL_RenderClear(renderer);
        setControlOutline();

        // get window size for positioning the red box
        int windowWidth, windowHeight;
        SDL_GetWindowSize(window, &windowWidth, &windowHeight);
        float redBoxX = windowWidth - RED_BOX_SIZE - RED_BOX_MARGIN;
        float redBoxY = windowHeight - RED_BOX_SIZE - RED_BOX_MARGIN;

        // render red box for PID tuning toggle
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_FRect redBox = {redBoxX, redBoxY, RED_BOX_SIZE, RED_BOX_SIZE};
        SDL_RenderFillRect(renderer, &redBox);

        // if the red box is clicked do all the things
        if (pid_tuning) renderParameterBoxes(proportional, integral, derivative, font, renderer);

        // check for any SDL window events
        while  (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) {
                windowRunning = false;
            }
            else if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    float mouseX = e.button.x;
                    float mouseY = e.button.y;

                    // check if red box was clicked
                    if (mouseX >= redBoxX && mouseX <= redBoxX + RED_BOX_SIZE &&
                        mouseY >= redBoxY && mouseY <= redBoxY + RED_BOX_SIZE) {
                        pid_tuning = true;
                        std::cout << "PID Tuning enabled" << std::endl;
                    }

                    for (int i = 0; i < 3; i++) {
                        float boxX = BOX_START_X + i * (BOX_WIDTH + BOX_SPACING);
                        float boxY = BOX_START_Y;
                        if (pid_tuning) {
                            if (mouseX >= boxX && mouseX <= boxX + BOX_WIDTH &&
                                mouseY >= boxY && mouseY <= boxY + BOX_HEIGHT) {
                                if (i == 0) proportional += 1.0f;
                                else if (i == 1) integral += 1.0f;
                                else if (i == 2) derivative += 1.0f;
                                box_clicked = true;
                            }
                        }
                    }
                }
                if (e.button.button == SDL_BUTTON_RIGHT) {
                    float mouseX = e.button.x;
                    float mouseY = e.button.y;

                    for (int i = 0; i < 3; i++) {
                        float boxX = BOX_START_X + i * (BOX_WIDTH + BOX_SPACING);
                        float boxY = BOX_START_Y;
                        if (pid_tuning) {
                            if (mouseX >= boxX && mouseX <= boxX + BOX_WIDTH &&
                                mouseY >= boxY && mouseY <= boxY + BOX_HEIGHT) {
                                if (i == 0) proportional -= 1.0f;
                                else if (i == 1) integral -= 1.0f;
                                else if (i == 2) derivative -= 1.0f;
                                box_clicked = true;
                        }
                        }
                    }
                }
            }
        }

        // scans the whole keyboard to check the states of input
        const bool* keyStates = SDL_GetKeyboardState(NULL);
        bool w_pressed = keyStates[SDL_SCANCODE_W]; // if the w key is pressed
        bool s_pressed = keyStates[SDL_SCANCODE_S];
        bool a_pressed = keyStates[SDL_SCANCODE_A];
        bool d_pressed = keyStates[SDL_SCANCODE_D];
        bool space_pressed = keyStates[SDL_SCANCODE_SPACE];
        bool shift_pressed = keyStates[SDL_SCANCODE_LSHIFT];
        bool control_pressed = keyStates[SDL_SCANCODE_LCTRL];

        // these are the components of the direction vector instruction to be converted into setpoint measurements
        // (in radians) that will be sent to the drone
        controlVector control_vectors;
        control_vectors.roll = 0;
        control_vectors.pitch = 0;
        control_vectors.yaw = 0;

        if (w_pressed && !s_pressed) {
            control_vectors.pitch = 127; // set a control vector component to FORWARD direction
            moveForward();
        }
        if (s_pressed && !w_pressed) {
            control_vectors.pitch = -127; // set a control vector component to BACKWARD direction
            moveBackward();
        }
        if (a_pressed && !d_pressed) { 
            control_vectors.roll = -127; // set a control vector component to LEFT direction
            moveLeft(); 
        }
        if (d_pressed && !a_pressed) {
            control_vectors.roll = 127; // set a control vector component to RIGHT direction
            moveRight();
        }
        if (shift_pressed) {
            if (throttleLevel < 251) throttleLevel += 4;
        }
        if (control_pressed) {
            if (throttleLevel > 4) throttleLevel -= 4;
        }
        if (keyStates[SDL_SCANCODE_ESCAPE]) windowRunning = false;
        if (keyStates[SDL_SCANCODE_0]) {
            throttleLevel = 0;
            std::cout << "MOTORS KILLED!\n";
            motors_killed = true;
        } 
        /////// CONVERT VECTORS TO SETPOINTS ///////////////////////////////////////////////////////
        // these are the new setpoint values that should be sent to the drone after recieving controller input
        float roll_setpoint = convertVectorToSetpoint(control_vectors.roll);
        float pitch_setpoint = convertVectorToSetpoint(control_vectors.pitch);
        float yaw_setpoint = 0;

        /////// SEND UDP DATA //////////////////////////////////////////////////////////////////////
        // udp_data is an array that holds all control data to be sent to the drone. The drone
        // will recognize the bytes in a set format defined below, so it must match on both programs!
        // the first element is what to change the throttle to
        // the second and third element is what vector the drone should aim to change its direction of travel to

        udpPacket packet = { 'c', throttleLevel, roll_setpoint, pitch_setpoint, yaw_setpoint };
        int len = sizeof(packet);
        sendto(sock, (const char*)&packet, len, 0, (sockaddr*)&addr, sizeof(addr));

        if (box_clicked && pid_tuning) {
            udpPacket pidPacket = { 'p', 0, proportional, integral, derivative };
            int pidlen = sizeof(pidPacket);
            sendto(sock, (const char*)&pidPacket, pidlen, 0, (sockaddr*)&addr, sizeof(addr));
            std::cout << "PACKET CONTENTS: "
                << pidPacket.identifier
                << " " << (int)pidPacket.throttle 
                << " " << pidPacket.roll
                << " " << pidPacket.pitch 
                << " " << pidPacket.yaw
                << std::dec << std::endl;
        }

        std::cout << "PACKET CONTENTS: "
            << packet.identifier
            << " " << (int)packet.throttle 
            << " " << packet.roll
            << " " << packet.pitch 
            << " " << packet.yaw
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
        
        SDL_RenderPresent(renderer);
        SDL_Delay(32);

        if (motors_killed) {
            windowRunning = false;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
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