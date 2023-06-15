#include <stdio.h>
#include <stdbool.h>

#include <SDL2/SDL.h>

#include "vec2.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 450

#define FPS 30
#define FRAME_TARGET_TIME (1000 / FPS)

bool game_is_running = false;
SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;

int last_frame_time = 0;

struct game_object {
	float x;
	float y;
	float velocity_x;
	float velocity_y;
	float width;
	float height;
	int color[3];
} ball, paddle;

void set_color_white(int color[])
{
	color[0] = 255;
	color[1] = 255;
	color[2] = 255;
}

void set_color_red(int color[])
{
	color[0] = 255;
	color[1] = 0;
	color[2] = 0;
}

bool check_collision()
{
	if ((ball.x < (paddle.x + paddle.width)) &&
	    ((ball.x + ball.width) > paddle.x) &&
	    (ball.y < (ball.y + paddle.height)) &&
	    ((ball.y + ball.height) > paddle.y))
		return true;
	else
       		return false;	
}

bool initialize_window(void)
{
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
		fprintf(stderr, "Error initializing SDL.\n");
		return false;
	}

	window = SDL_CreateWindow(
		NULL, 
		SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED,
		WINDOW_WIDTH,
		WINDOW_HEIGHT,
		SDL_WINDOW_BORDERLESS
	);

	if (!window) {
		fprintf(stderr, "Error creating SDL Window.\n");
		return false;
	}

	renderer = SDL_CreateRenderer(window, -1, 0);
	if (!renderer) {
		fprintf(stderr, "Error creating SDL Renderer.\n");
		return false;
	}

	return true;
}

void process_input() {
	SDL_Event event;
	SDL_PollEvent(&event);

	paddle.velocity_x = 0;

	switch (event.type) {
		case SDL_QUIT:
			game_is_running = false;
			break;
		case SDL_KEYDOWN:
			if (event.key.keysym.sym == SDLK_ESCAPE)
				game_is_running = false;
			if (event.key.keysym.sym == SDLK_LEFT)
			{
				set_color_red(paddle.color);
				paddle.velocity_x = -100;
			}
			else if (event.key.keysym.sym == SDLK_RIGHT)
			{
				set_color_red(paddle.color);
				paddle.velocity_x = 100;
			}
			break;
		case SDL_KEYUP:
			if (event.key.keysym.sym == SDLK_LEFT)
			{
				set_color_white(paddle.color);
				paddle.velocity_x = 0;
			}
			else if (event.key.keysym.sym == SDLK_RIGHT)
			{
				set_color_white(paddle.color);
				paddle.velocity_x = 0;
			}
	}
}

void setup() {
	ball.x = 20;
	ball.y = 20;
	ball.width = 15;
	ball.height = 15;
	ball.velocity_x = 200;
	ball.velocity_y = 200;
	set_color_white(ball.color);

	paddle.x = WINDOW_WIDTH / 2.0 - paddle.width / 2.0;
	paddle.y = WINDOW_HEIGHT - 50;
	paddle.width = 80;
	paddle.height = 15;
	paddle.velocity_x = 0;
	paddle.velocity_y = 0;
	set_color_white(paddle.color);
}


void update() {

	int time_to_wait = FRAME_TARGET_TIME - (SDL_GetTicks() - last_frame_time);

	if (time_to_wait > 0 && time_to_wait <= FRAME_TARGET_TIME) {
		SDL_Delay(time_to_wait);
	}

	float delta_time = (SDL_GetTicks() - last_frame_time) / 1000.0f;

	last_frame_time = SDL_GetTicks();

	ball.x += ball.velocity_x * delta_time;
	ball.y += ball.velocity_y * delta_time;

	paddle.x += paddle.velocity_x * delta_time;

	if (ball.y > (WINDOW_HEIGHT - ball.height))
	{
		ball.velocity_y *= -1;
	}
	if (ball.x > (WINDOW_WIDTH - ball.width))
	{
		ball.velocity_x *= -1;
	}
	if (ball.y < 0)
	{
		ball.velocity_y *= -1;
	}
	if (ball.x < 0)
	{
		ball.velocity_x *= -1;
	}

	if (paddle.x < 0)
	{
		paddle.x = 0;
	}
	if (paddle.x > (WINDOW_WIDTH - paddle.width))
	{
		paddle.x = WINDOW_WIDTH - paddle.width;
	}

	if (check_collision())
	{
		ball.velocity_y *= -1;
	}

}

void render() {
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);

	SDL_Rect ball_rect = {
		(int)ball.x,
		(int)ball.y,
		(int)ball.width,
		(int)ball.height
	};

	SDL_Rect paddle_rect = {
		(int)paddle.x,
		(int)paddle.y,
		(int)paddle.width,
		(int)paddle.height
	};

	SDL_SetRenderDrawColor(renderer, ball.color[0], ball.color[1], ball.color[2], 255);
	SDL_RenderFillRect(renderer, &ball_rect);

	SDL_SetRenderDrawColor(renderer, paddle.color[0], paddle.color[1], paddle.color[2], 255);
	SDL_RenderFillRect(renderer, &paddle_rect);

	SDL_RenderPresent(renderer);
}

void destroy_window() {
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}

int main(void)
{
	game_is_running = initialize_window();

	setup();

	while (game_is_running) {
		process_input();
		update();
		render();
	}

	destroy_window();

	return 0;
}
