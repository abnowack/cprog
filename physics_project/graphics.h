#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdio.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
#include "vec2.h"
#include "mem.h"

typedef struct
{
    int window_width;
    int window_height;
    SDL_Window *window;
    SDL_Renderer *renderer;
} Graphics;

Graphics gfx = {0, 0, NULL, NULL};

bool gfx_create_window(int window_width, int window_height)
{
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        fprintf(stderr, "Error initializing SDL\n");
        return false;
    }
    SDL_DisplayMode display_mode;
    SDL_GetCurrentDisplayMode(0, &display_mode);
    gfx.window_width = display_mode.w;
    gfx.window_height = display_mode.h;
    gfx.window = SDL_CreateWindow(NULL, 0, 0, gfx.window_width, gfx.window_height, SDL_WINDOW_BORDERLESS);
    if (!gfx.window)
    {
        fprintf(stderr, "Error creating SDL window\n");
        return false;
    }
    gfx.renderer = SDL_CreateRenderer(gfx.window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!gfx.renderer)
    {
        fprintf(stderr, "Error creating SDL renderer\n");
        return false;
    }

    SDL_SetRenderDrawColor(gfx.renderer, 255, 255, 255, 255);
    SDL_RenderClear(gfx.renderer);

    return true;
}

void gfx_close_window()
{
    SDL_DestroyRenderer(gfx.renderer);
    SDL_DestroyWindow(gfx.window);
    SDL_Quit();
}

void gfx_clear_screen(uint8_t color[3])
{
    SDL_SetRenderDrawColor(gfx.renderer, 255, 255, 255, 255);
    SDL_RenderClear(gfx.renderer);
}

void gfx_render_frame()
{
    SDL_RenderPresent(gfx.renderer);
}

void gfx_draw_filled_square(int x, int y, int width, uint8_t color[3])
{
    // x, y is center of rect
    SDL_SetRenderDrawColor(gfx.renderer, color[0], color[1], color[2], 255);

    int upper_left_x = x - (int)(width / 2.0);
    int upper_left_y = y - (int)(width / 2.0);

    SDL_Rect r = {.x = upper_left_x, .y = upper_left_y, .w = width, .h = width};
    SDL_RenderFillRect(gfx.renderer, &r);
}

void gfx_draw_circle(int x, int y, int radius, float angle, uint8_t color[3])
{
    unsigned int n_slices = 100;

    mem_reset_scratch_pool();
    SDL_Point *points = (SDL_Point *)mem_calloc(n_slices, sizeof(SDL_Point), MEM_SCRATCH_POOL);
    for (unsigned int i = 0; i < n_slices; i++)
    {
        float radian = (2.0 * M_PI * i) / (n_slices - 1);
        float local_x = radius * cosf(radian);
        float local_y = radius * sinf(radian);

        int global_x = x + (int)local_x;
        int global_y = y + (int)local_y;

        points[i] = (SDL_Point){.x = global_x, .y = global_y};
    }
    SDL_SetRenderDrawColor(gfx.renderer, color[0], color[1], color[2], 255);
    SDL_RenderDrawLines(gfx.renderer, points, n_slices);

    SDL_RenderDrawLine(gfx.renderer, x, y, (int)(x + radius * cosf(angle)), (int)(y + radius * sinf(angle)));
    gfx_draw_filled_square(x, y, 8, color);
}

void gfx_draw_line(int x0, int y0, int x1, int y1, uint8_t color[3])
{
    SDL_SetRenderDrawColor(gfx.renderer, color[0], color[1], color[2], 255);
    SDL_RenderDrawLine(gfx.renderer, x0, y0, x1, y1);
}

void gfx_draw_polygon(int x, int y, Vec2 *vertices, unsigned int n_vertices, uint8_t color[3])
{
    mem_reset_scratch_pool();
    SDL_Point *points = (SDL_Point *)mem_calloc(n_vertices + 1, sizeof(SDL_Point), MEM_SCRATCH_POOL);
    for (unsigned int i = 0; i < n_vertices; i++)
    {
        points[i] = (SDL_Point){.x = (int)vertices[i].x, (int)vertices[i].y};
    }
    points[n_vertices] = (SDL_Point){.x = (int)vertices[0].x, .y = (int)vertices[0].y};

    SDL_SetRenderDrawColor(gfx.renderer, color[0], color[1], color[2], 255);
    SDL_RenderDrawLines(gfx.renderer, points, n_vertices + 1);

    gfx_draw_filled_square(x, y, 8, color);
}

#endif