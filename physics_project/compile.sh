#!/bin/bash

gcc -g -std=c99 $1.c -lSDL2 -lm -lSDL2_image -lSDL2_gfx -lGL -o $1 
