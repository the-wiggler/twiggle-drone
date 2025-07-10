#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

constexpr float MAX_DRONE_TILT = 0.2; // the maximum amount of radians the drone can tilt when being controlled

struct controlVector {
    int8_t roll;
    int8_t pitch;
    int8_t yaw;
};
struct udpPacket {
    uint8_t throttle;
    float roll;
    float pitch;
    float yaw;
};

constexpr uint8_t motorIdleSpeed = 50;

SDL_Window* window = SDL_CreateWindow("Drone Controller", 1000, 1000, SDL_WINDOW_RESIZABLE);
SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);



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

#endif