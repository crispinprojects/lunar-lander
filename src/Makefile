# Makefile for OpenGL Lunar Lander Simulation
# Usage:  make        (builds 'lander3d')
#         make run    (runs the demo)
#         make clean  (removes build products)

CC      = gcc
CFLAGS  = -Wall -O2
LIBS    = -lSDL3 -lGL -lGLU -lm -lglut
SRC     = main.c lander3d.c
OUT     = lander3d

$(OUT): $(SRC)
	$(CC) $(CFLAGS) -o $(OUT) $(SRC) $(LIBS)

run: $(OUT)
	./$(OUT)

clean:
	rm -f $(OUT)
