//
// Created by oguzh on 11.01.2026.
//

#include "render_2d.h"

#include <SDL.h>
#include <SDL_image.h>
#include <iostream>

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

    if (!(IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG)) {
        std::cerr << "IMG_Init failed: " << IMG_GetError() << std::endl;
    }

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

void render_2d::loadTexture(const std::string& path)
{
    SDL_Surface* surface = IMG_Load(path.c_str());
    if (!surface) {
        std::cerr << "IMG_Load failed: " << IMG_GetError() << std::endl;
        return;
    }
    m_playerTexture = SDL_CreateTextureFromSurface(m_renderer, surface);
    SDL_FreeSurface(surface);
    if (!m_playerTexture) {
        std::cerr << "SDL_CreateTextureFromSurface failed: " << SDL_GetError() << std::endl;
    }
}

void render_2d::shutdown()
{
    if (m_playerTexture) {
        SDL_DestroyTexture(m_playerTexture);
        m_playerTexture = nullptr;
    }

    if (m_renderer) {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }

    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }

    IMG_Quit();
    SDL_Quit();
}

void render_2d::handleEvents()
{
    SDL_PumpEvents();
}

void render_2d::render(PhysicsWorld& world)
{
    if (!isValid())
        return;

    handleEvents();

    draw_frame(
        m_renderer,
        world,
        m_playerTexture
    );
}
