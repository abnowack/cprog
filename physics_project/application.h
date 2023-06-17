#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL2/SDL.h>
#include "graphics.h"
#include "body.h"
#include "force.h"

#define FPS 60
#define MILLISECONDS_PER_FRAME ((int)(1000.0f / FPS))
#define PIXELS_PER_METER 50

#define MAX_BODIES 10

struct Application
{
    bool running;

    Body b[MAX_BODIES];
    unsigned int n_bodies;

    vec2 mouse_cursor_pos;
    bool mouse_button_down;
};

int time_previous_frame;

struct Application app = {.running = false};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);
    app.n_bodies = 0;
    time_previous_frame = 0;
    app.mouse_cursor_pos = (vec2){0, 0};
    app.mouse_button_down = false;

    // Circle *c = (Circle *)malloc(sizeof(Circle));
    // *c = circle_create(50.0);
    // app.b[app.n_bodies] = body_create((Shape*)c, gfx.window_width / 2.0, gfx.window_height / 2.0, 1.0);
    // app.n_bodies++;

    Box *b = (Box *)malloc(sizeof(Box));
    *b = box_create(200, 100);
    app.b[app.n_bodies] = body_create(BOX, b, gfx.window_width / 2.0, gfx.window_height / 2.0, 1.0);
    app.n_bodies++;
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
        case SDL_KEYUP:
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
                // if (app.n_Bodys < MAX_BodyS)
                // {
                //     app.p[app.n_Bodys] = Body_create(x, y, 1.0);
                //     app.p[app.n_Bodys].radius = 4;
                //     app.p[app.n_Bodys].frozen = true;
                //     app.n_Bodys++;
                // }
            }
            break;
        case SDL_MOUSEBUTTONUP:
            if (app.mouse_button_down && event.button.button == SDL_BUTTON_LEFT)
            {
                app.mouse_button_down = false;
                // vec2 diff = vec2_sub(app.p[app.n_Bodys - 1].position, app.mouse_cursor_pos);
                // vec2 impulse_dir = vec2_unitvector(diff);
                // float impulse_magnitude = vec2_norm(diff) * 5.0;
                // app.p[app.n_Bodys - 1].velocity = vec2_scale(impulse_dir, impulse_magnitude);
                // app.p[app.n_Bodys - 1].frozen = false;
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
    for (unsigned int i = 0; i < app.n_bodies; i++)
    {
        // vec2 wind = {2.0 * PIXELS_PER_METER, 0.0};
        // Body_add_force(&(app.p[i]), wind);
        // vec2 gravity = {0.0, app.b[i].mass * 9.8 * PIXELS_PER_METER};
        // body_add_force(&(app.b[i]), gravity);

        float torque = 2000;
        body_add_torque(&(app.b[i]), torque);
    }

    for (unsigned int i = 0; i < app.n_bodies; i++)
    {
        body_update(&(app.b[i]), delta_time);
    }

    for (unsigned int i = 0; i < app.n_bodies; i++)
    {
        if (app.b[i].shape_type == CIRCLE)
        {
            Circle *c = (Circle *)(app.b[i].shape);
            if (app.b[i].position.x - c->radius <= 0)
            {
                app.b[i].position.x = c->radius;
                app.b[i].velocity.x *= -1.0;
            }
            else if (app.b[i].position.x + c->radius >= gfx.window_width)
            {
                app.b[i].position.x = gfx.window_width - c->radius;
                app.b[i].velocity.x *= -1.0;
            }
            if (app.b[i].position.y - c->radius <= 0)
            {
                app.b[i].position.y = c->radius;
                app.b[i].velocity.y *= -1.0;
            }
            else if (app.b[i].position.y + c->radius >= gfx.window_height)
            {
                app.b[i].position.y = gfx.window_height - c->radius;
                app.b[i].velocity.y *= -1.0;
            }
        }
    }
}

void app_render()
{
    gfx_clear_screen((uint8_t[3]){255, 255, 255});

    for (unsigned int i = 0; i < app.n_bodies; i++)
    {
        if (app.b[i].shape_type == CIRCLE)
        {
            Circle *c = (Circle *)(app.b[i].shape);
            gfx_draw_circle(app.b[i].position.x, app.b[i].position.y, c->radius, app.b[i].theta, (uint8_t[3]){255, 0, 255});
        }
        else if (app.b[i].shape_type == BOX)
        {
            Box *b = (Box *)(app.b[i].shape);
            gfx_draw_polygon(app.b[i].position.x, app.b[i].position.y, b->global_vertices, b->n_vertices, (uint8_t[3]){255, 0, 255});
        }
        else if (app.b[i].shape_type == POLYGON)
        {
        }
        else
        {
        }
    }

    // if (app.mouse_button_down)
    // {
    // gfx_draw_line(app.p[app.n_Bodys - 1].position.x, app.p[app.n_Bodys - 1].position.y, app.mouse_cursor_pos.x, app.mouse_cursor_pos.y, (uint8_t[3]){255, 0, 0});
    // }

    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif