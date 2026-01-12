//
// Created by oguzh on 12.01.2026.
//

#include "debug_draw.h"

#include "physics_world.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <SDL_rect.h>

// ------------------------------------------------------------
// Camera / scale policy
// ------------------------------------------------------------
// Scale is based on window width to avoid HiDPI / WSL saturation.
static constexpr float VISIBLE_WORLD_WIDTH = 20.0f;

static SDL_Point world_to_screen(
    float wx, float wy,
    int screen_w, int screen_h,
    float ppm)
{
    SDL_Point p;
    p.x = static_cast<int>(screen_w * 0.5f + wx * ppm);
    p.y = static_cast<int>(screen_h * 0.5f - wy * ppm);
    return p;
}

// ------------------------------------------------------------
// Low-level primitives
// ------------------------------------------------------------
static void draw_point(
    SDL_Renderer* renderer,
    float wx, float wy,
    int screen_w, int screen_h,
    float ppm,
    int radius = 3)
{
    SDL_Point p = world_to_screen(wx, wy, screen_w, screen_h, ppm);

    SDL_Rect r;
    r.x = p.x - radius;
    r.y = p.y - radius;
    r.w = radius * 4;
    r.h = radius * 4;

    SDL_RenderFillRect(renderer, &r);
}

static void draw_line(
    SDL_Renderer* renderer,
    float ax, float ay,
    float bx, float by,
    int screen_w, int screen_h,
    float ppm)
{
    SDL_Point a = world_to_screen(ax, ay, screen_w, screen_h, ppm);
    SDL_Point b = world_to_screen(bx, by, screen_w, screen_h, ppm);

    SDL_RenderDrawLine(renderer, a.x, a.y, b.x, b.y);
}

static void draw_aabb(
    SDL_Renderer* renderer,
    float cx, float cy,
    float hx, float hy,
    int screen_w, int screen_h,
    float ppm)
{
    SDL_Point min = world_to_screen(cx - hx, cy - hy, screen_w, screen_h, ppm);
    SDL_Point max = world_to_screen(cx + hx, cy + hy, screen_w, screen_h, ppm);

    SDL_Rect r;
    r.x = min.x;
    r.y = max.y;
    r.w = max.x - min.x;
    r.h = min.y - max.y;

    SDL_RenderDrawRect(renderer, &r);
}

static void draw_vector(
    SDL_Renderer* renderer,
    float ox, float oy,
    float vx, float vy,
    int screen_w, int screen_h,
    float ppm,
    float scale)
{
    draw_line(
        renderer,
        ox, oy,
        ox + vx * scale,
        oy + vy * scale,
        screen_w, screen_h,
        ppm
    );
}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
void debug_draw_body(
    SDL_Renderer* renderer,
    const Body& body,
    int screen_w,
    int screen_h,
    float ppm)
{

    // ---- AABB ----
    draw_aabb(
        renderer,
        body.position.x,
        body.position.y,
        body.halfWidth,
        body.halfHeight,
        screen_w,
        screen_h,
        ppm
    );


    // ---- AABB color policy ----
    if (body.invMass == 0.0f) {
        // Static body
        SDL_SetRenderDrawColor(renderer, 180, 180, 180, 255);
    }
    else if (body.onGround) {
        // Grounded dynamic
        SDL_SetRenderDrawColor(renderer, 100, 255, 100, 255);
    }
    else {
        // Airborne dynamic
        SDL_SetRenderDrawColor(renderer, 100, 200, 255, 255);
    }

    draw_point(
        renderer,
        body.position.x,
        body.position.y,
        screen_w,
        screen_h,
        ppm
    );

    // ---- Velocity ----
    if (body.invMass != 0.0f) {
        SDL_SetRenderDrawColor(renderer, 255, 100, 100, 255);
        draw_vector(
            renderer,
            body.position.x,
            body.position.y,
            body.velocity.x,
            body.velocity.y,
            screen_w,
            screen_h,
            ppm,
            0.25f
        );
    }
}

void draw_frame(
    SDL_Renderer* renderer,
    PhysicsWorld& world)
{
    SDL_RenderSetLogicalSize(renderer, 0, 0);
    SDL_RenderSetScale(renderer, 1.0f, 1.0f);
    SDL_RenderSetViewport(renderer, nullptr);
    int rw, rh;
    SDL_GetRendererOutputSize(renderer, &rw, &rh);

    SDL_Rect vp;
    SDL_RenderGetViewport(renderer, &vp);

    float sx, sy;
    SDL_RenderGetScale(renderer, &sx, &sy);

    std::cout
    << "[DRAW] output=" << rw << "x" << rh
    << " viewport=("
    << vp.x << "," << vp.y << " " << vp.w << "x" << vp.h
    << ") scale=("
    << std::fixed << std::setprecision(2)
    << sx << ", " << sy
    << ")"
    << std::endl;
    fflush(stdout);
    SDL_RenderSetViewport(renderer, nullptr);
    int screen_w = 0;
    int screen_h = 0;
    SDL_GetRendererOutputSize(renderer, &screen_w, &screen_h);
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_RenderClear(renderer);

    const float ppm = screen_w / VISIBLE_WORLD_WIDTH;

    // ---- Ground reference line (y = 0) ----
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
    draw_line(renderer, -10.0f, 0.0f, 10.0f, 0.0f, screen_w, screen_h, ppm);


    // ---- Bodies ----
    for (const Body& body : world.getBodies()) {
        debug_draw_body(renderer, body, screen_w, screen_h, ppm);
    }

    SDL_RenderPresent(renderer);
}
