//
// Created by oguzh on 12.01.2026.
//

#ifndef DEBUGDRAW_H
#define DEBUGDRAW_H


#include <SDL.h>
#include "physics_world.h"
#include "body.h"

void draw_frame(
    SDL_Renderer* renderer,
    PhysicsWorld& world,
    SDL_Texture* playerTex = nullptr
);

void debug_draw_body(
    SDL_Renderer* renderer,
    const Body& body,
    int screen_w,
    int screen_h,
    float pixels_per_meter
);




#endif //DEBUGDRAW_H
