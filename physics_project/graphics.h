#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdio.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include "vec2.h"

struct Graphics
{
    int window_width;
    int window_height;
    SDL_Window *window;
    SDL_Renderer *renderer;
};

struct Graphics gfx = {0, 0, NULL, NULL};

bool gfx_create_window(int window_width, int window_height)
{
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        fprintf(stderr, "Error initializing SDL\n");
        return false;
    }
    SDL_DisplayMode display_mode;
    SDL_GetCurrentDisplayMode(0, &display_mode);
    gfx.window_width = window_width;
    gfx.window_height = window_height;
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
    SDL_SetRenderDrawColor(gfx.renderer, color[0], color[1], color[2], 255);
    SDL_RenderClear(gfx.renderer);
}

void gfx_render_frame()
{
    SDL_RenderPresent(gfx.renderer);
}

void gfx_draw_filled_circle(int x, int y, int radius, uint8_t color[3])
{
    uint32_t rgba = (255 << 24) + (color[2] << 16) + (color[1] << 8) + color[0];
    filledCircleColor(gfx.renderer, x, y, radius, rgba);
}

void gfx_draw_circle(int x, int y, int radius, float angle, uint8_t color[3])
{
    uint32_t rgba = (255 << 24) + (color[2] << 16) + (color[1] << 8) + color[0];
    circleColor(gfx.renderer, x, y, radius, rgba);
    lineColor(gfx.renderer, x, y, x + cosf(angle) * radius, y + sinf(angle) * radius, rgba);
}

void gfx_draw_filled_rectangle(int x, int y, int width, int height, uint8_t color[3])
{
    SDL_SetRenderDrawColor(gfx.renderer, color[0], color[1], color[2], 255);
    SDL_RenderFillRect(gfx.renderer, &(SDL_Rect){.x = x, .y = y, .w = width, .h = height});
}

void gfx_draw_line(int x0, int y0, int x1, int y1, uint8_t color[3])
{
    uint32_t rgba = (255 << 24) + (color[2] << 16) + (color[1] << 8) + color[0];
    lineColor(gfx.renderer, x0, y0, x1, y1, rgba);
}

void gfx_draw_polygon(int x, int y, vec2 *vertices, unsigned int n_vertices, uint8_t color[3])
{
    uint32_t rgba = (255 << 24) + (color[2] << 16) + (color[1] << 8) + color[0];
    for (unsigned int i = 0; i < n_vertices; i++)
    {
        unsigned int next_index = (i + 1) % n_vertices;
        lineColor(gfx.renderer, vertices[i].x, vertices[i].y, vertices[next_index].x, vertices[next_index].y, rgba);
    }
    filledCircleColor(gfx.renderer, x, y, 1, rgba);
}

#endif