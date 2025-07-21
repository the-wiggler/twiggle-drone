#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <iomanip>

constexpr float MAX_DRONE_TILT = 0.2; // the maximum amount of radians the drone can tilt when being controlled

struct controlVector {
    int8_t roll;
    int8_t pitch;
    int8_t yaw;
};
struct udpPacket {
    char identifier;
    uint32_t throttle;
    float roll;
    float pitch;
    float yaw;
};

constexpr uint32_t motorMaxSpeed = 1023; // max bit value
constexpr uint32_t motorIdleSpeed = motorMaxSpeed * 0.25;

SDL_Window* window = SDL_CreateWindow("Drone Controller", 1000, 1000, SDL_WINDOW_RESIZABLE);
SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);

// function to convert throttle value to percentage
uint32_t percentageToThrottle(float percentage) {
    if (percentage <= 0.0f) {
        return 0; 
    }
    if (percentage >= 100.0f) {
        return motorMaxSpeed;
    }
    float range = motorMaxSpeed - motorIdleSpeed;
    return motorIdleSpeed + (uint32_t)((percentage / 100.0f) * range);
}

void moveForward() {
    SDL_FRect upRect = {475, 75, 50, 100};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderFillRect(renderer, &upRect);
}

void moveBackward() {
    SDL_FRect upRect = {475, 275, 50, 100};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderFillRect(renderer, &upRect);
}

void moveLeft() {
    SDL_FRect upRect = {350, 200, 100, 50};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderFillRect(renderer, &upRect);
}

void moveRight() {
    SDL_FRect upRect = {550, 200, 100, 50};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderFillRect(renderer, &upRect);
}

void setControlOutline() {
    SDL_FRect outline1 = {475, 75, 50, 100};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 50);
    SDL_RenderFillRect(renderer, &outline1);

    SDL_FRect outline2 = {475, 275, 50, 100};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 50);
    SDL_RenderFillRect(renderer, &outline2);

    SDL_FRect outline3 = {350, 200, 100, 50};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 50);
    SDL_RenderFillRect(renderer, &outline3);

    SDL_FRect outline4 = {550, 200, 100, 50};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 50);
    SDL_RenderFillRect(renderer, &outline4);
}

// this converts the control vector components to radian setpoint values
// that are usable on the PID system of the drone
float convertVectorToSetpoint(int8_t vectorInput) {
    return (vectorInput / 127) * MAX_DRONE_TILT;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // box dimensions and positions
    constexpr float BOX_WIDTH = 80.0f;
    constexpr float BOX_HEIGHT = 30.0f;
    constexpr float BOX_SPACING = 10.0f;
    constexpr float BOX_START_X = 10.0f;
    constexpr float BOX_START_Y = 10.0f;

    // this is the box to activate PID tuning
    const float RED_BOX_SIZE = 30.0f;
    const float RED_BOX_MARGIN = 20.0f;

void renderParameterBoxes(float param1, float param2, float param3, TTF_Font* font, SDL_Renderer* renderer) {
    SDL_Color whiteColor = {255, 255, 255, 255};
    SDL_Color boxColor = {60, 60, 60, 255};
    SDL_Color borderColor = {100, 100, 100, 255};
    
    for (int i = 0; i < 3; i++) {
        float x = BOX_START_X + i * (BOX_WIDTH + BOX_SPACING);
        float y = BOX_START_Y;
        
        // draw box background
        SDL_FRect boxRect = {x, y, BOX_WIDTH, BOX_HEIGHT};
        SDL_SetRenderDrawColor(renderer, boxColor.r, boxColor.g, boxColor.b, boxColor.a);
        SDL_RenderFillRect(renderer, &boxRect);
        
        // Draw box border
        SDL_SetRenderDrawColor(renderer, borderColor.r, borderColor.g, borderColor.b, borderColor.a);
        SDL_RenderRect(renderer, &boxRect);
        
        // prepare text
        float value = (i == 0) ? param1 : (i == 1) ? param2 : param3;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(0) << value;
        std::string text = ss.str();
        
        // render text
        SDL_Surface* textSurface = TTF_RenderText_Blended(font, text.c_str(), text.length(), whiteColor);
        if (textSurface) {
            SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
            SDL_FRect textRect = {
                x + (BOX_WIDTH - textSurface->w) / 2,
                y + (BOX_HEIGHT - textSurface->h) / 2,
                (float)textSurface->w,
                (float)textSurface->h
            };
            SDL_RenderTexture(renderer, textTexture, NULL, &textRect);
            SDL_DestroyTexture(textTexture);
            SDL_DestroySurface(textSurface);
        }
    }
}

#endif