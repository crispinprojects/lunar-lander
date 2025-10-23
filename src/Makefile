# Compiler and flags
CC = gcc
CFLAGS = -Wall -g $(shell pkg-config --cflags sdl3)
LDFLAGS = $(shell pkg-config --libs sdl3) -lm -lSDL3_image -lSDL3_ttf

# Source and target
SRC = main.c
OBJ = main.o
TARGET = lunar

# Default target to build the application
all: $(TARGET)

# Compile the object file from the source file
$(OBJ): $(SRC)
	$(CC) $(CFLAGS) -c $(SRC)

# Link the object file to create the executable
$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Clean up the build files
clean:
	rm -f $(OBJ) $(TARGET) main

run:
	./$(TARGET) 

# Declare phony targets (these aren't files)
.PHONY: all clean run
