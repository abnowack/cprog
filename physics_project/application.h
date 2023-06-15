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

    Particle p;
};

int time_previous_frame;

struct Application app = {.running = false};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);

    time_previous_frame = 0;

    app.p = particle_create(50, 100, 1.0);
    app.p.velocity = (vec2){50.0, 10.0};
    app.p.radius = 4;
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
    app.p.acceleration = (vec2){0, 9.8 * PIXELS_PER_METER};

    vec2 dv = vec2_scale(app.p.acceleration, delta_time);
    app.p.velocity = vec2_add(app.p.velocity, dv);
    vec2 dx = vec2_scale(app.p.velocity, delta_time);
    app.p.position = vec2_add(app.p.position, dx);

    if (app.p.position.x - app.p.radius <= 0)
    {
        app.p.position.x = app.p.radius;
        app.p.velocity.x *= -1.0;
    } 
    else if (app.p.position.x + app.p.radius >= gfx.window_width)
    {
        app.p.position.x = gfx.window_width - app.p.radius;
        app.p.velocity.x *= -1.0;
    }
    if (app.p.position.y - app.p.radius <= 0)
    {
        app.p.position.y = app.p.radius;
        app.p.velocity.y *= -1.0;
    } 
    else if (app.p.position.y + app.p.radius >= gfx.window_height)
    {
        app.p.position.y = gfx.window_height - app.p.radius;
        app.p.velocity.y *= -1.0;
    }
}

void app_render()
{
    gfx_clear_screen((uint8_t [3]){255, 255, 255});
    gfx_draw_filled_circle(app.p.position.x, app.p.position.y, app.p.radius, (uint8_t [3]){255, 0, 255});
    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif