# Makefile for OpenGL Lunar Lander Simulation
# Usage:  make        (builds 'lander3d')
#         make run    (runs the demo)
#         make clean  (cleans)

CC      = gcc
CFLAGS  = -Wall -O2
LIBS    = -lGL -lGLU -lm -lglut
SRC     = main.c lander3d.c particles.c
OUT     = lander3d

$(OUT): $(SRC)
	$(CC) $(CFLAGS) -o $(OUT) $(SRC) $(LIBS)

run: $(OUT)
	./$(OUT)

clean:
	rm -f $(OUT)
