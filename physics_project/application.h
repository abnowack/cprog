#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL2/SDL.h>
#include "graphics.h"
#include "world.h"
#include "body.h"
#include "force.h"
#include "collision.h"

#include "mem.h"

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
    Body *b1 = (Body*)malloc(sizeof(Body));
    *b1 = body_create(CIRCLE, c1, gfx.window_width / 2, gfx.window_height / 2, 0.0);
    List_push(&app.world.bodies, b1);

    Polygon *box = (Polygon *)malloc(sizeof(Polygon));
    *box = box_create(800, 50);
    Body *b2 = (Body*)malloc(sizeof(Body));
    *b2 = body_create(BOX, box, b1->position.x, b1->position.y + 200, 1.0);
    b2->friction = 0.5;
    b2->restitution = 0.5;
    List_push(&app.world.bodies, b2);

    JointConstraint *jc = (JointConstraint*)malloc(sizeof(JointConstraint));
    joint_constraint_create(jc, b1, b2, b1->position);
    List_push(&app.world.joint_constraints, jc);

    Circle *c2 = (Circle *)malloc(sizeof(Circle));
    *c2 = circle_create(20.0);
    Body *b3 = (Body*)malloc(sizeof(Body));
    *b3 = body_create(CIRCLE, c2, b2->position.x, b2->position.y + 150, 1.0);
    List_push(&app.world.bodies, b3);

    jc = (JointConstraint*)malloc(sizeof(JointConstraint));
    joint_constraint_create(jc, b2, b3, b2->position);
    List_push(&app.world.joint_constraints, jc);

    Circle *c3 = (Circle *)malloc(sizeof(Circle));
    *c3 = circle_create(20.0);
    Body *b4 = (Body*)malloc(sizeof(Body));
    *b4 = body_create(CIRCLE, c3, b3->position.x, b3->position.y + 150, 1.0);
    List_push(&app.world.bodies, b4);

    jc = (JointConstraint*)malloc(sizeof(JointConstraint));
    joint_constraint_create(jc, b3, b4, b3->position);
    List_push(&app.world.joint_constraints, jc);

    Polygon *floor = (Polygon *)malloc(sizeof(Polygon));
    *floor = box_create(gfx.window_width - 50, 25);
    Body *b5 = (Body*)malloc(sizeof(Body));
    *b5 = body_create(BOX, floor, gfx.window_width / 2.0, gfx.window_height - 25, 0.0);
    b5->restitution = 0.6;
    b5->friction = 0.5;
    List_push(&app.world.bodies, b5);

    Polygon *left_wall = (Polygon *)malloc(sizeof(Polygon));
    *left_wall = box_create(25, gfx.window_height - 50);
    Body *b6 = (Body*)malloc(sizeof(Body));
    *b6 = body_create(BOX, left_wall, 12, gfx.window_height / 2.0 + 12, 0.0);
    b6->restitution = 0.6;
    b6->friction = 0.5;
    List_push(&app.world.bodies, b6);

    Polygon *right_wall = (Polygon *)malloc(sizeof(Polygon));
    *right_wall = box_create(25, gfx.window_height - 50);
    Body *b7 = (Body*)malloc(sizeof(Body));
    *b7 = body_create(BOX, right_wall, gfx.window_width - 12, gfx.window_height / 2.0 + 12, 0.0);
    b7->restitution = 0.6;
    List_push(&app.world.bodies, b7);
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
                if (app.new_shape_type == CIRCLE)
                {
                    Circle *c = (Circle *)malloc(sizeof(Circle));
                    *c = circle_create(100.0);
                    Body *b = (Body*)malloc(sizeof(Body));
                    *b = body_create(CIRCLE, c, x, y, 1.0);
                    b->restitution = 0.6;
                    b->friction = 0.4;
                    List_push(&app.world.bodies, b);
                }
                else if (app.new_shape_type == BOX)
                {
                    Polygon *p = (Polygon *)malloc(sizeof(Polygon));
                    *p = box_create(100, 100);
                    Body *b = (Body*)malloc(sizeof(Body));
                    *b = body_create(BOX, p, x, y, 1.0);
                    b->restitution = 0.6;
                    b->friction = 0.4;
                    List_push(&app.world.bodies, b);
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
                    Body *b = (Body*)malloc(sizeof(Body));
                    *b = body_create(POLYGON, p, x, y, 1.0);
                    b->restitution = 0.6;
                    b->friction = 0.7;
                    List_push(&app.world.bodies, b);
                }
            }
            break;
        case SDL_MOUSEBUTTONUP:
            if (app.mouse_button_down && event.button.button == SDL_BUTTON_LEFT)
            {
                app.mouse_button_down = false;
            }
            break;
        }
    }
}

void app_update()
{
    mem_log.heap_memory_allocated = 0;
    mem_log.heap_memory_calls = 0;
    gfx_clear_screen((uint8_t[3]){255, 255, 255});

    int time_to_wait = MILLISECONDS_PER_FRAME - (SDL_GetTicks() - time_previous_frame);
    if (time_to_wait > 0)
    {
        SDL_Delay(time_to_wait);
    }

    float delta_time = (SDL_GetTicks() - time_previous_frame) / 1000.0f;
    if (delta_time > (1.0f / (float)FPS))
    {
        printf("%f > %f\n", delta_time, (1.0f / (float)FPS));
        delta_time = (1.0f / (float)FPS);
    }

    time_previous_frame = SDL_GetTicks();

    world_update(&app.world, delta_time);
    // printf("[MEM] %zu bytes allocated this update, %zu calls\n", mem_log.heap_memory_allocated, mem_log.heap_memory_calls);
}

void app_render()
{
    // gfx_clear_screen((uint8_t[3]){255, 255, 255});

    uint8_t not_collide_color[3] = {0, 0, 255};
    uint8_t collide_color[3] = {255, 0, 0};

    for (Node *n = app.world.bodies.start, *next; n; n = next)
    {
        uint8_t draw_color[3] = {not_collide_color[0], not_collide_color[1], not_collide_color[2]};

        // if (app.world.b[i].is_colliding && app.debug)
        // {
        //     draw_color[0] = collide_color[0];
        //     draw_color[1] = collide_color[1];
        //     draw_color[2] = collide_color[2];
        // }

        Body *b = (Body*)n->data;

        if (b->shape_type == CIRCLE)
        {
            Circle *c = (Circle *)(b->shape);
            gfx_draw_circle(b->position.x, b->position.y, c->radius, b->theta, draw_color);
        }
        else if (b->shape_type == BOX || b->shape_type == POLYGON)
        {
            Polygon *p = (Polygon *)(b->shape);
            gfx_draw_polygon(b->position.x, b->position.y, p->global_vertices, p->n_vertices, draw_color);
        }
        else
        {
        }
        next = n->next;
    }

    for(Node *n = app.world.joint_constraints.start, *next; n; n = next)
    {
        JointConstraint *jc = (JointConstraint*)n->data;
        Vec2 pa = body_local_to_global_space(jc->a, jc->a_local_anchor);
        Vec2 pb = body_local_to_global_space(jc->b, jc->a_local_anchor);
        gfx_draw_line(pa.x, pa.y, pb.x, pb.y, collide_color);
        next = n->next;
    }

    gfx_render_frame();
}

void app_destroy()
{
    gfx_close_window();
}

#endif