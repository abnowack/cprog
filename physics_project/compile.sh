#!/bin/bash

gcc -g -std=c99 $1.c -lSDL2 -lm -lSDL2_image -lSDL2_gfx -lGL -o $1 -pg
# gcc -std=c99 -O3 $1.c -lSDL2 -lm -lSDL2_image -lSDL2_gfx -lGL -o $1g
