#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL2/SDL.h>
#include "graphics.h"
#include "world.h"
#include "body.h"
#include "force.h"
#include "collision.h"

#define FPS 60
#define MILLISECONDS_PER_FRAME ((int)(1000.0f / FPS))

typedef struct
{
    bool running;
    bool debug;
    World world;

    Vec2 mouse_cursor_pos;
    bool mouse_button_down;

    ShapeType new_shape_type;
} Application;

int time_previous_frame;

Application app = {.running = false};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);
    app.debug = false;
    world_create(&app.world, -9.8f);
    time_previous_frame = 0;
    app.mouse_cursor_pos = (Vec2){0, 0};
    app.mouse_button_down = false;
    app.new_shape_type = CIRCLE;

    Circle *c1 = (Circle *)malloc(sizeof(Circle));
    *c1 = circle_create(30.0);
    app.world.b[app.world.n_bodies++] = body_create(CIRCLE, c1, gfx.window_width / 2, gfx.window_height / 2, 0.0);

    Polygon *box = (Polygon *)malloc(sizeof(Polygon));
    *box = box_create(800, 50);
    app.world.b[app.world.n_bodies] = body_create(BOX, box, app.world.b[0].position.x, app.world.b[0].position.y + 200, 1.0);
    app.world.b[app.world.n_bodies].friction = 0.5;
    app.world.b[app.world.n_bodies].restitution = 0.5;
    app.world.n_bodies++;

    joint_constraint_create(&app.world.joint_constraints[app.world.n_joint_constraints], &app.world.b[0], &app.world.b[1], app.world.b[0].position);
    app.world.n_joint_constraints++;

    Circle *c2 = (Circle *)malloc(sizeof(Circle));
    *c2 = circle_create(20.0);
    app.world.b[app.world.n_bodies++] = body_create(CIRCLE, c2, app.world.b[1].position.x, app.world.b[1].position.y + 150, 1.0);

    joint_constraint_create(&app.world.joint_constraints[app.world.n_joint_constraints], &app.world.b[1], &app.world.b[2], app.world.b[1].position);
    app.world.n_joint_constraints++;

    Circle *c3 = (Circle *)malloc(sizeof(Circle));
    *c3 = circle_create(20.0);
    app.world.b[app.world.n_bodies++] = body_create(CIRCLE, c3, app.world.b[2].position.x, app.world.b[2].position.y + 150, 1.0);

    joint_constraint_create(&app.world.joint_constraints[app.world.n_joint_constraints], &app.world.b[2], &app.world.b[3], app.world.b[2].position);
    app.world.n_joint_constraints++;

    Polygon *floor = (Polygon *)malloc(sizeof(Polygon));
    *floor = box_create(gfx.window_width - 50, 25);
    app.world.b[app.world.n_bodies] = body_create(BOX, floor, gfx.window_width / 2.0, gfx.window_height - 25, 0.0);
    app.world.b[app.world.n_bodies].restitution = 0.2;
    app.world.b[app.world.n_bodies].friction = 0.5;
    app.world.n_bodies++;

    Polygon *left_wall = (Polygon *)malloc(sizeof(Polygon));
    *left_wall = box_create(25, gfx.window_height - 50);
    app.world.b[app.world.n_bodies] = body_create(BOX, left_wall, 12, gfx.window_height / 2.0 + 12, 0.0);
    app.world.b[app.world.n_bodies].restitution = 0.2;
    app.world.b[app.world.n_bodies].friction = 0.5;
    app.world.n_bodies++;

    Polygon *right_wall = (Polygon *)malloc(sizeof(Polygon));
    *right_wall = box_create(25, gfx.window_height - 50);
    app.world.b[app.world.n_bodies] = body_create(BOX, right_wall, gfx.window_width - 12, gfx.window_height / 2.0 + 12, 0.0);
    app.world.b[app.world.n_bodies].restitution = 0.2;
    app.world.n_bodies++;
}

void app_input()
{

    SDL_Event event;
    Body b;
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
            if (event.key.keysym.sym == SDLK_d)
                app.debug = !app.debug;
            if (event.key.keysym.sym == SDLK_1)
                app.new_shape_type = CIRCLE;
            if (event.key.keysym.sym == SDLK_2)
                app.new_shape_type = BOX;
            if (event.key.keysym.sym == SDLK_3)
                app.new_shape_type = POLYGON;
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
                if (app.world.n_bodies < MAX_BODIES)
                {
                    if (app.new_shape_type == CIRCLE)
                    {
                        Circle *c = (Circle *)malloc(sizeof(Circle));
                        *c = circle_create(100.0);
                        app.world.b[app.world.n_bodies] = body_create(CIRCLE, c, x, y, 1.0);
                        app.world.b[app.world.n_bodies].restitution = 0.3;
                        app.world.b[app.world.n_bodies].friction = 0.4;
                        app.world.n_bodies++;
                    }
                    else if (app.new_shape_type == BOX)
                    {
                        Polygon *p = (Polygon *)malloc(sizeof(Polygon));
                        *p = box_create(100, 100);
                        app.world.b[app.world.n_bodies] = body_create(BOX, p, x, y, 1.0);
                        app.world.b[app.world.n_bodies].restitution = 0.3;
                        app.world.b[app.world.n_bodies].friction = 0.4;
                        app.world.n_bodies++;
                    }
                    else if (app.new_shape_type == POLYGON)
                    {
                        Polygon *p = (Polygon *)malloc(sizeof(Polygon));
                        Vec2 points[5];
                        points[0] = (Vec2){20, 60};
                        points[1] = (Vec2){-40, 20};
                        points[2] = (Vec2){-20, -60};
                        points[3] = (Vec2){20, -60};
                        points[4] = (Vec2){40, 20};
                        *p = polygon_create(points, 5);
                        app.world.b[app.world.n_bodies] = body_create(POLYGON, p, x, y, 1.0);
                        app.world.b[app.world.n_bodies].restitution = 0.1;
                        app.world.b[app.world.n_bodies].friction = 0.7;
                        app.world.n_bodies++;
                    }
                }
            }
            break;
        case SDL_MOUSEBUTTONUP:
            if (app.mouse_button_down && event.button.button == SDL_BUTTON_LEFT)
            {
                app.mouse_button_down = false;
                // Vec2 diff = vec2_sub(app.p[app.n_Bodys - 1].position, app.mouse_cursor_pos);
                // Vec2 impulse_dir = vec2_unitvector(diff);
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
    gfx_clear_screen((uint8_t[3]){255, 255, 255});

    int time_to_wait = MILLISECONDS_PER_FRAME - (SDL_GetTicks() - time_previous_frame);
    if (time_to_wait > 0)
    {
        SDL_Delay(time_to_wait);
    }

    float delta_time = (SDL_GetTicks() - time_previous_frame) / 1000.0f;
    if (delta_time > 0.016)
    {
        delta_time = 0.016;
    }

    time_previous_frame = SDL_GetTicks();

    world_update(&app.world, delta_time);
}

void app_render()
{
    // gfx_clear_screen((uint8_t[3]){255, 255, 255});

    uint8_t not_collide_color[3] = {0, 0, 255};
    uint8_t collide_color[3] = {255, 0, 0};

    for (unsigned int i = 0; i < app.world.n_bodies; i++)
    {
        uint8_t draw_color[3] = {not_collide_color[0], not_collide_color[1], not_collide_color[2]};

        // if (app.world.b[i].is_colliding && app.debug)
        // {
        //     draw_color[0] = collide_color[0];
        //     draw_color[1] = collide_color[1];
        //     draw_color[2] = collide_color[2];
        // }

        if (app.world.b[i].shape_type == CIRCLE)
        {
            Circle *c = (Circle *)(app.world.b[i].shape);
            gfx_draw_circle(app.world.b[i].position.x, app.world.b[i].position.y, c->radius, app.world.b[i].theta, draw_color);
        }
        else if (app.world.b[i].shape_type == BOX || app.world.b[i].shape_type == POLYGON)
        {
            Polygon *b = (Polygon *)(app.world.b[i].shape);
            gfx_draw_polygon(app.world.b[i].position.x, app.world.b[i].position.y, b->global_vertices, b->n_vertices, draw_color);
        }
        else
        {
        }
    }

    for (unsigned int i = 0; i < app.world.n_joint_constraints; i++)
    {
        Vec2 pa = body_local_to_global_space(app.world.joint_constraints[i].a, app.world.joint_constraints[i].a_local_anchor);
        Vec2 pb = body_local_to_global_space(app.world.joint_constraints[i].b, app.world.joint_constraints[i].a_local_anchor);
        gfx_draw_line(pa.x, pa.y, pb.x, pb.y, collide_color);
    }

    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif