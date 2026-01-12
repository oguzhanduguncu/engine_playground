//
// Created by oguzh on 11.01.2026.
//

#include "render_2d.h"

#include <SDL.h>

#include "physics_world.h"
#include "debug_draw.h"

render_2d::render_2d(int width, int height)
{
    init(width, height);
}

render_2d::~render_2d()
{
    shutdown();
}

bool render_2d::isValid() const
{
    return m_window && m_renderer;
}

bool render_2d::isRunning() const
{
    return m_running;
}

void render_2d::init(int width, int height)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
        return;

    m_window = SDL_CreateWindow(
        "Simulation Prototype",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        width,
        height,
        SDL_WINDOW_RESIZABLE
    );

    if (!m_window)
        return;

    m_renderer = SDL_CreateRenderer(
        m_window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (!m_renderer)
        return;

    m_screenW = width;
    m_screenH = height;

    SDL_RenderSetLogicalSize(m_renderer, 0, 0);
    SDL_RenderSetScale(m_renderer, 1.0f, 1.0f);
    SDL_RenderSetViewport(m_renderer, nullptr);

    SDL_GetRendererOutputSize(m_renderer, &m_screenW, &m_screenH);


    SDL_Rect vp{0, 0, m_screenW, m_screenH};
    SDL_RenderSetViewport(m_renderer, &vp);
}

void render_2d::shutdown()
{
    if (m_renderer) {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }

    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }

    SDL_Quit();
}

void render_2d::handleEvents()
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) {
            m_running = false;
        }
        else if (e.type == SDL_WINDOWEVENT &&
                 e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {

            handleResize();
        }
    }
}

void render_2d::handleResize()
{
    if (m_renderer) {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }

    m_renderer = SDL_CreateRenderer(
        m_window,
        -1,
        SDL_RENDERER_SOFTWARE
    );

    SDL_RenderSetLogicalSize(m_renderer, 0, 0);
    SDL_RenderSetScale(m_renderer, 1.0f, 1.0f);
    SDL_RenderSetViewport(m_renderer, nullptr);
}

void render_2d::render(PhysicsWorld& world)
{
    if (!isValid())
        return;

    draw_frame(
        m_renderer,
        world
    );
}
