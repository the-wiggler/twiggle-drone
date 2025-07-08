#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

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


#endif