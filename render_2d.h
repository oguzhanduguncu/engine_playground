//
// Created by oguzh on 11.01.2026.
//

#ifndef RENDER2D_H
#define RENDER2D_H
#include <SDL_render.h>
#include <string>


class PhysicsWorld;

class render_2d {
public:
    render_2d() = default;
    render_2d(int width, int height);
    ~render_2d();

    bool isValid() const;
    bool isRunning() const;

    void handleEvents();
    void render(PhysicsWorld& world);

private:
    void init(int width, int height);
    void shutdown();

    void handleResize();

    SDL_Window* m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;

    bool m_running = true;

    int m_screenW = 0;
    int m_screenH = 0;
};



#endif //RENDER2D_H
