#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL2/SDL.h>
#include "graphics.h"
#include "particle.h"

#define FPS 60
#define MILLISECONDS_PER_FRAME ((int)(1000.0f / FPS))
#define PIXELS_PER_METER 50

struct Application
{
    bool running;

    Particle p[2];
};

int time_previous_frame;

struct Application app = {.running = false};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);

    time_previous_frame = 0;

    app.p[0] = particle_create(50, 100, 1.0);
    app.p[0].radius = 4;

    app.p[1] = particle_create(50, 200, 3.0);
    app.p[1].radius = 12;
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
    int time_to_wait = MILLISECONDS_PER_FRAME - (SDL_GetTicks() - time_previous_frame);
    if (time_to_wait > 0)
    {
        SDL_Delay(time_to_wait);
    }

    float delta_time = (SDL_GetTicks() - time_previous_frame) / 1000.0f;
    if (delta_time > 0.0333)
    {
        delta_time = 0.0333;
    }

    time_previous_frame = SDL_GetTicks();

    // physics
    vec2 wind = {1.0 * PIXELS_PER_METER, 0.0};
    for (unsigned int i = 0; i < 2; i++)
    {
        particle_add_force(&(app.p[i]), wind);
    }

    for (unsigned int i = 0; i < 2; i++)
    {
        particle_integrate(&(app.p[i]), delta_time);
    }

    for (unsigned int i = 0; i < 2; i++)
    {
        if (app.p[i].position.x - app.p[i].radius <= 0)
        {
            app.p[i].position.x = app.p[i].radius;
            app.p[i].velocity.x *= -1.0;
        } 
        else if (app.p[i].position.x + app.p[i].radius >= gfx.window_width)
        {
            app.p[i].position.x = gfx.window_width - app.p[i].radius;
            app.p[i].velocity.x *= -1.0;
        }
        if (app.p[i].position.y - app.p[i].radius <= 0)
        {
            app.p[i].position.y = app.p[i].radius;
            app.p[i].velocity.y *= -1.0;
        } 
        else if (app.p[i].position.y + app.p[i].radius >= gfx.window_height)
        {
            app.p[i].position.y = gfx.window_height - app.p[i].radius;
            app.p[i].velocity.y *= -1.0;
        }
    }
}

void app_render()
{
    gfx_clear_screen((uint8_t [3]){255, 255, 255});
    for (unsigned int i = 0; i < 2; i++)
    {
        gfx_draw_filled_circle(app.p[i].position.x, app.p[i].position.y, app.p[i].radius, (uint8_t [3]){255, 0, 255});
    }
    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif