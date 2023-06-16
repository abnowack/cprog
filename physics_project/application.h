#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL2/SDL.h>
#include "graphics.h"
#include "particle.h"
#include "force.h"

#define FPS 60
#define MILLISECONDS_PER_FRAME ((int)(1000.0f / FPS))
#define PIXELS_PER_METER 50

#define MAX_PARTICLES 50

struct Application
{
    bool running;

    Particle p[MAX_PARTICLES];
    unsigned int n_particles;

    vec2 push_force;
    vec2 mouse_cursor_pos;
    bool mouse_button_down;
};

int time_previous_frame;

struct Application app = {.running = false, .push_force = (vec2){0, 0}};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);
    app.n_particles = 0;
    time_previous_frame = 0;
    app.mouse_cursor_pos = (vec2){0, 0};
    app.mouse_button_down = false;

    app.p[app.n_particles] = particle_create(50, 100, 1.0);
    app.p[app.n_particles].radius = 4;
    app.p[app.n_particles].velocity = (vec2){10 * PIXELS_PER_METER, 10 * PIXELS_PER_METER};
    app.n_particles++;

    app.p[app.n_particles] = particle_create(50, 200, 3.0);
    app.p[app.n_particles].radius = 12;
    app.n_particles++;
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
        case SDL_MOUSEMOTION:
            app.mouse_cursor_pos.x = event.motion.x;
            app.mouse_cursor_pos.y = event.motion.y;
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (!app.mouse_button_down && event.button.button == SDL_BUTTON_LEFT)
            {
                app.mouse_button_down = true;
                int x, y;
                SDL_GetMouseState(&x, &y);
                app.mouse_cursor_pos.x = x;
                app.mouse_cursor_pos.y = y;
                if (app.n_particles < MAX_PARTICLES)
                {
                    app.p[app.n_particles] = particle_create(x, y, 1.0);
                    app.p[app.n_particles].radius = 4;
                    app.p[app.n_particles].frozen = true;
                    app.n_particles++;
                }

            }
            break;
        case SDL_MOUSEBUTTONUP:
            if (app.mouse_button_down && event.button.button == SDL_BUTTON_LEFT)
            {
                app.mouse_button_down = false;
                vec2 diff = vec2_sub(app.p[app.n_particles - 1].position, app.mouse_cursor_pos);
                vec2 impulse_dir = vec2_unitvector(diff);
                float impulse_magnitude = vec2_norm(diff) * 5.0;
                app.p[app.n_particles - 1].velocity = vec2_scale(impulse_dir, impulse_magnitude);
                app.p[app.n_particles - 1].frozen = false;
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
    for (unsigned int i = 0; i < app.n_particles; i++)
    {
        // vec2 wind = {2.0 * PIXELS_PER_METER, 0.0};
        // particle_add_force(&(app.p[i]), wind);
        // vec2 gravity = {0.0, app.p[i].mass * 9.8 * PIXELS_PER_METER};
        // particle_add_force(&(app.p[i]), gravity);

        particle_add_force(&(app.p[i]), app.push_force);

        // vec2 friction = force_friction(&(app.p[i]), 10.0 * PIXELS_PER_METER);
        // particle_add_force(&(app.p[i]), friction);

        for (unsigned int j = i+1; j < app.n_particles; j++)
        {
            if (app.p[i].frozen || app.p[j].frozen)
                continue;

            float G = 10000.0;
            vec2 gravity = force_gravity(&(app.p[i]), &(app.p[j]), G, 5, 100);
            particle_add_force(&(app.p[i]), gravity);
            particle_add_force(&(app.p[j]), vec2_scale(gravity, -1.0));
        }
    }

    for (unsigned int i = 0; i < app.n_particles; i++)
    {
        if (!app.p[i].frozen)
            particle_integrate(&(app.p[i]), delta_time);
    }

    for (unsigned int i = 0; i < app.n_particles; i++)
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

    for (unsigned int i = 0; i < app.n_particles; i++)
    {
        gfx_draw_filled_circle(app.p[i].position.x, app.p[i].position.y, app.p[i].radius, (uint8_t[3]){255, 0, 255});
    }

    if (app.mouse_button_down)
    {
        gfx_draw_line(app.p[app.n_particles - 1].position.x, app.p[app.n_particles - 1].position.y, app.mouse_cursor_pos.x, app.mouse_cursor_pos.y, (uint8_t[3]){255, 0, 0});
    }

    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif