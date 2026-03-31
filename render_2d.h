//
// Created by oguzh on 11.01.2026.
//

#ifndef RENDER2D_H
#define RENDER2D_H
#include <SDL.h>
#include <string>


class PhysicsWorld;
class Flock;

class render_2d {
public:
    render_2d() = default;
    render_2d(int width, int height);
    ~render_2d();

    bool isValid() const;
    bool isRunning() const;

    void handleEvents();
    bool wasKeyPressed(SDL_Keycode key) const { return m_lastKey == key; }
    void render(PhysicsWorld& world);
    void render(PhysicsWorld& world, Flock& flock);

    void loadTexture(const std::string& path);
    [[nodiscard]] SDL_Texture* playerTexture() const { return m_playerTexture; }

private:
    void init(int width, int height);
    void shutdown();

    SDL_Window* m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;
    SDL_Texture* m_playerTexture = nullptr;

    bool       m_running = true;
    SDL_Keycode m_lastKey = SDLK_UNKNOWN;

    int m_screenW = 0;
    int m_screenH = 0;
};



#endif //RENDER2D_H
