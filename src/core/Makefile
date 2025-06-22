CC = gcc
CFLAGS = -Wall -Wextra -pedantic

all: flight_control

flight_control: src/core/flight_control.c
    $(CC) $(CFLAGS) -o bin/flight_control $^

clean:
    rm -f bin/*
