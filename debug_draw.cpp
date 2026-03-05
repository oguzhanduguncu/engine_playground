//
// Created by oguzh on 12.01.2026.
//

#include "debug_draw.h"

#include "physics_world.h"
#include "boid_flock.h"
#include <glm/glm.hpp>
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
    Shape body_shape,
    int screen_w, int screen_h,
    float ppm,
    int radius = 3)
{
    SDL_Point p = world_to_screen(wx, wy, screen_w, screen_h, ppm);

    SDL_Rect r;
    r.x = p.x - radius*3;
    r.y = p.y - radius*3;
    r.w = radius * 3;
    r.h = radius * 3;

    if (body_shape.type == Type::plane)
    {
        // draw_line(renderer, -10.0f, 0.0f, 10.0f, 0.0f, screen_w, screen_h, ppm);
        SDL_Point a = world_to_screen(wx-10.0f, wy, screen_w, screen_h, ppm);
        SDL_Point b = world_to_screen(wx+10.0f, wy, screen_w, screen_h, ppm);

        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
        SDL_RenderDrawLine(renderer, a.x, a.y, b.x, b.y);
        return;
    }

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
        body.shape,
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

static void draw_boid_shape(SDL_Renderer*, const Boid&, int, int, float);

void draw_frame(
    SDL_Renderer* renderer,
    PhysicsWorld& world,
    const Flock* flock,
    SDL_Texture* playerTex)
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

    /*std::cout
    << "[DRAW] output=" << rw << "x" << rh
    << " viewport=("
    << vp.x << "," << vp.y << " " << vp.w << "x" << vp.h
    << ") scale=("
    << std::fixed << std::setprecision(2)
    << sx << ", " << sy
    << ")"
    << std::endl;*/
    fflush(stdout);
    SDL_RenderSetViewport(renderer, nullptr);
    int screen_w = 0;
    int screen_h = 0;
    SDL_GetRendererOutputSize(renderer, &screen_w, &screen_h);
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_RenderClear(renderer);

    const float ppm = screen_w / VISIBLE_WORLD_WIDTH;

    // ---- Bodies ----
    for (const Body& body : world.getBodies()) {
        debug_draw_body(renderer, body, screen_w, screen_h, ppm);
        // Draw cat texture on top for player body (id 5)
        if (body.id == 5 && playerTex) {
            float bottom_y = PhysicsWorld::GROUND_Y + body.halfHeight;
            float top_y = bottom_y + body.halfHeight * 2.0f;
            SDL_Point min = world_to_screen(
                body.position.x - body.halfWidth,
                bottom_y,
                screen_w, screen_h, ppm);
            SDL_Point max = world_to_screen(
                body.position.x + body.halfWidth,
                top_y,
                screen_w, screen_h, ppm);
            SDL_Rect dst;
            dst.x = min.x;
            dst.y = max.y;
            dst.w = max.x - min.x;
            dst.h = min.y - max.y;
            SDL_RenderCopy(renderer, playerTex, nullptr, &dst);
        }
    }

    // ---- Boids ----
    if (flock) {
        for (const Boid& b : flock->getBoids())
            draw_boid_shape(renderer, b, screen_w, screen_h, ppm);
    }

    SDL_RenderPresent(renderer);
}

// Draw a single boid as a triangle pointing in its velocity direction
static void draw_boid_shape(
    SDL_Renderer* renderer,
    const Boid& boid,
    int screen_w, int screen_h,
    float ppm)
{
    float px = boid.body.position.x;
    float py = boid.body.position.y;

    glm::vec2 fwd{1.0f, 0.0f};
    float spd = glm::length(boid.body.velocity);
    if (spd > 0.001f) fwd = boid.body.velocity / spd;
    glm::vec2 right{-fwd.y, fwd.x};

    constexpr float TIP_LEN    = 0.4f;
    constexpr float BASE_BACK  = 0.2f;
    constexpr float BASE_HALF  = 0.15f;

    glm::vec2 tip   = {px + fwd.x * TIP_LEN,  py + fwd.y * TIP_LEN};
    glm::vec2 left  = {px - fwd.x * BASE_BACK + right.x * BASE_HALF,
                       py - fwd.y * BASE_BACK + right.y * BASE_HALF};
    glm::vec2 rght  = {px - fwd.x * BASE_BACK - right.x * BASE_HALF,
                       py - fwd.y * BASE_BACK - right.y * BASE_HALF};

    SDL_Point s_tip  = world_to_screen(tip.x,  tip.y,  screen_w, screen_h, ppm);
    SDL_Point s_left = world_to_screen(left.x, left.y, screen_w, screen_h, ppm);
    SDL_Point s_rght = world_to_screen(rght.x, rght.y, screen_w, screen_h, ppm);

    SDL_SetRenderDrawColor(renderer, 255, 220, 50, 255);
    SDL_RenderDrawLine(renderer, s_tip.x,  s_tip.y,  s_left.x, s_left.y);
    SDL_RenderDrawLine(renderer, s_left.x, s_left.y, s_rght.x, s_rght.y);
    SDL_RenderDrawLine(renderer, s_rght.x, s_rght.y, s_tip.x,  s_tip.y);
}

