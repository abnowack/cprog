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

struct Application
{
    bool running;
    bool debug;
    World world;

    vec2 mouse_cursor_pos;
    bool mouse_button_down;

    ShapeType new_shape_type;
};

int time_previous_frame;

struct Application app = {.running = false};

void app_setup(int window_width, int window_height)
{
    app.running = gfx_create_window(window_width, window_height);
    app.debug = false;
    world_create(&app.world, -9.8f);
    time_previous_frame = 0;
    app.mouse_cursor_pos = (vec2){0, 0};
    app.mouse_button_down = false;
    app.new_shape_type = CIRCLE;


    Circle *c1 = (Circle *)malloc(sizeof(Circle));
    Body *body1 = (Body *)malloc(sizeof(Body));
    *c1 = circle_create(30.0);
    *body1 = body_create(CIRCLE, c1, gfx.window_width / 2, gfx.window_height / 2, 0.0);
    world_add_body(&app.world, body1);

    Polygon *box = (Polygon *)malloc(sizeof(Polygon));
    Body *boxb = (Body *)malloc(sizeof(Body));
    *box = box_create(800, 50);
    *boxb = body_create(BOX, box, app.world.b[0].position.x, app.world.b[0].position.y + 200, 1.0);
    boxb->friction = 0.5;
    boxb->restitution = 0.5;
    world_add_body(&app.world, boxb);

    JointConstraint *jc = (JointConstraint*)malloc(sizeof(JointConstraint));
    joint_constraint_create(jc, &app.world.b[0], &app.world.b[1], app.world.b[0].position);
    world_add_joint_constraints(&app.world, jc);

    Circle *c2 = (Circle *)malloc(sizeof(Circle));
    Body *body3 = (Body *)malloc(sizeof(Body));
    *c2 = circle_create(20.0);
    *body3 = body_create(CIRCLE, c2, app.world.b[1].position.x, app.world.b[1].position.y + 150, 1.0);
    world_add_body(&app.world, body3);

    JointConstraint *jc2 = (JointConstraint*)malloc(sizeof(JointConstraint));
    joint_constraint_create(jc2, &app.world.b[1], &app.world.b[2], app.world.b[1].position);
    world_add_joint_constraints(&app.world, jc2);

    Circle *c3 = (Circle *)malloc(sizeof(Circle));
    Body *body4 = (Body *)malloc(sizeof(Body));
    *c3 = circle_create(20.0);
    *body4 = body_create(CIRCLE, c3, app.world.b[2].position.x, app.world.b[2].position.y + 150, 1.0);
    world_add_body(&app.world, body4);

    JointConstraint *jc3 = (JointConstraint*)malloc(sizeof(JointConstraint));
    joint_constraint_create(jc3, &app.world.b[2], &app.world.b[3], app.world.b[2].position);
    world_add_joint_constraints(&app.world, jc3);

    // Circle *c4 = (Circle *)malloc(sizeof(Circle));
    // Body *body5 = (Body *)malloc(sizeof(Body));
    // *c4 = circle_create(20.0);
    // *body5 = body_create(CIRCLE, c4, app.world.b[3].position.x, app.world.b[3].position.y + 70, 1.0);
    // world_add_body(&app.world, body5);

    // JointConstraint *jc4 = (JointConstraint*)malloc(sizeof(JointConstraint));
    // joint_constraint_create(jc4, &app.world.b[3], &app.world.b[4], app.world.b[3].position);
    // world_add_joint_constraints(&app.world, jc4);

    // *c = circle_create(20.0);
    // body = body_create(CIRCLE, c, app.world.b[4].position.x, app.world.b[4].position.y + 70, 1.0);
    // world_add_body(&app.world, &body);

    // JointConstraint *jc5 = (JointConstraint*)malloc(sizeof(JointConstraint));
    // joint_constraint_create(jc5, &app.world.b[4], &app.world.b[5], app.world.b[4].position);
    // world_add_joint_constraints(&app.world, jc5);

    Polygon *floor = (Polygon *)malloc(sizeof(Polygon));
    Body *body_floor = (Body *)malloc(sizeof(Body));
    *floor = box_create(gfx.window_width - 50, 25);
    *body_floor = body_create(BOX, floor, gfx.window_width / 2.0, gfx.window_height - 25, 0.0);
    body_floor->restitution = 0.2;
    body_floor->friction = 0.5;
    world_add_body(&app.world, body_floor);

    Polygon *left_wall = (Polygon *)malloc(sizeof(Polygon));
    Body *body_left_wall = (Body *)malloc(sizeof(Body));
    *left_wall = box_create(25, gfx.window_height - 50);
    *body_left_wall = body_create(BOX, left_wall, 12, gfx.window_height / 2.0 + 12, 0.0);
    body_left_wall->restitution = 0.2;
    body_left_wall->friction = 0.5;
    world_add_body(&app.world, body_left_wall);

    Polygon *right_wall = (Polygon *)malloc(sizeof(Polygon));
    Body *body_right_wall = (Body *)malloc(sizeof(Body));
    *right_wall = box_create(25, gfx.window_height - 50);
    *body_right_wall = body_create(BOX, right_wall, gfx.window_width - 12, gfx.window_height / 2.0 + 12, 0.0);
    body_right_wall->restitution = 0.2;
    world_add_body(&app.world, body_right_wall);
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
                        *c = circle_create(40.0);
                        b = body_create(CIRCLE, c, x, y, 1.0);
                        b.restitution = 0.3;
                        b.friction = 0.4;
                        world_add_body(&app.world, &b);
                    }
                    else if (app.new_shape_type == BOX)
                    {
                        Polygon *p = (Polygon *)malloc(sizeof(Polygon));
                        *p = box_create(50, 50);
                        b = body_create(BOX, p, x, y, 1.0);
                        b.restitution = 0.3;
                        b.friction = 0.4;
                        world_add_body(&app.world, &b);
                    }
                    else if (app.new_shape_type == POLYGON)
                    {
                        Polygon *p = (Polygon *)malloc(sizeof(Polygon));
                        vec2 points[5];
                        points[0] = (vec2){20, 60};
                        points[1] = (vec2){-40, 20};
                        points[2] = (vec2){-20, -60};
                        points[3] = (vec2){20, -60};
                        points[4] = (vec2){40, 20};
                        *p = polygon_create(points, 5);
                        b = body_create(POLYGON, p, x, y, 1.0);
                        b.restitution = 0.1;
                        b.friction = 0.7;
                        world_add_body(&app.world, &b);
                    }
                }
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

        if (app.world.b[i].is_colliding && app.debug)
        {
            draw_color[0] = collide_color[0];
            draw_color[1] = collide_color[1];
            draw_color[2] = collide_color[2];
        }

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
        vec2 pa = body_local_to_global_space(app.world.joint_constraints[i]->a, app.world.joint_constraints[i]->a_local_anchor);
        vec2 pb = body_local_to_global_space(app.world.joint_constraints[i]->b, app.world.joint_constraints[i]->a_local_anchor);
        gfx_draw_line(pa.x, pa.y, pb.x, pb.y, collide_color);
    }

    gfx_render_frame();

}

void app_destroy()
{
    gfx_close_window();
}

#endif