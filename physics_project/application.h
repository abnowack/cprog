#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL2/SDL.h>
#include "graphics.h"

struct Application
{
    bool running;
};

struct Application app = {.running = false};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);
}

void app_input()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_QUIT:
            app.running = false;
            break;
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                app.running = false;
            break;
        }
    }
}

void app_update()
{

}

void app_render()
{
    gfx_clear_screen((uint8_t [3]){255, 255, 255});
    gfx_draw_filled_circle(200, 200, 40, (uint8_t [3]){255, 0, 255});
    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif