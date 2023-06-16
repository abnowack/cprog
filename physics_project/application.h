#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL2/SDL.h>
#include "graphics.h"
#include "particle.h"
#include "force.h"

#define FPS 60
#define MILLISECONDS_PER_FRAME ((int)(1000.0f / FPS))
#define PIXELS_PER_METER 50

struct Application
{
    bool running;

    Particle p[2];
    vec2 push_force;

    SDL_Rect liquid;
};

int time_previous_frame;

struct Application app = {.running = false, .push_force = (vec2){0, 0}};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);

    time_previous_frame = 0;

    app.p[0] = particle_create(50, 100, 1.0);
    app.p[0].radius = 4;

    app.p[1] = particle_create(50, 200, 3.0);
    app.p[1].radius = 12;

    app.liquid.x = 0;
    app.liquid.y = gfx.window_height / 2;
    app.liquid.w = gfx.window_width;
    app.liquid.h = gfx.window_height / 2;
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
            if (event.key.keysym.sym == SDLK_UP)
                app.push_force.y = -50 * PIXELS_PER_METER;
            if (event.key.keysym.sym == SDLK_RIGHT)
                app.push_force.x = 50 * PIXELS_PER_METER;
            if (event.key.keysym.sym == SDLK_DOWN)
                app.push_force.y = 50 * PIXELS_PER_METER;
            if (event.key.keysym.sym == SDLK_LEFT)
                app.push_force.x = -50 * PIXELS_PER_METER;
            break;
        case SDL_KEYUP:
            if (event.key.keysym.sym == SDLK_UP)
                app.push_force.y = 0;
            if (event.key.keysym.sym == SDLK_RIGHT)
                app.push_force.x = 0;
            if (event.key.keysym.sym == SDLK_DOWN)
                app.push_force.y = 0;
            if (event.key.keysym.sym == SDLK_LEFT)
                app.push_force.x = 0;
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_LEFT)
            {
                int x, y;
                SDL_GetMouseState(&x, &y);
                app.p[0].position = (vec2){x, y};
            }
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
    // apply forces
    for (unsigned int i = 0; i < 2; i++)
    {
        // vec2 wind = {2.0 * PIXELS_PER_METER, 0.0};
        // particle_add_force(&(app.p[i]), wind);
        // vec2 gravity = {0.0, app.p[i].mass * 9.8 * PIXELS_PER_METER};
        // particle_add_force(&(app.p[i]), gravity);

        particle_add_force(&(app.p[i]), app.push_force);

        // if (app.p[i].position.y >= app.liquid.y)
        // {
            // particle_add_force(&(app.p[i]), force_drag(&(app.p[i]), 0.01));
        // }

        vec2 friction = force_friction(&(app.p[i]), 10.0 * PIXELS_PER_METER);
        particle_add_force(&(app.p[i]), friction);
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
    gfx_clear_screen((uint8_t[3]){255, 255, 255});

    gfx_draw_filled_rectangle(
        app.liquid.x, 
        app.liquid.y, 
        app.liquid.w, 
        app.liquid.h,
        (uint8_t [3]){0x13, 0x37, 0x6E});

    for (unsigned int i = 0; i < 2; i++)
    {
        gfx_draw_filled_circle(app.p[i].position.x, app.p[i].position.y, app.p[i].radius, (uint8_t[3]){255, 0, 255});
    }
    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif