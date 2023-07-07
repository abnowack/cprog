#!/bin/bash

# gcc -g -std=c99 $1.c -lSDL2 -lm -lSDL2_image -lSDL2_gfx -lGL -o $1 -pg
# gcc -std=c99 -O3 $1.c -lSDL2 -lm -lSDL2_image -lSDL2_gfx -lGL -o $1g

# emcc -std=c99 -o $1.html $1.c -lm -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s SDL2_IMAGE_FORMATS='["png"]' --preload-file assets/ --use-preload-plugins
emcc $1.c -lm -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s SDL2_IMAGE_FORMATS='["png"]' -o $1.html --embed-file assets