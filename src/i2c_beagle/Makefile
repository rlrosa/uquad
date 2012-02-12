EXECUTABLE=cmd
CC=gcc
CFLAGS=-O
SOURCES=cmd_motores.c
all:
	$(CC) $(CFLAGS) cmd_motores.c -o $(EXECUTABLE)
debug:
	$(CC) -g $(SOURCES) -o $(EXECUTABLE)_debug
clean:
	rm $(EXECUTABLE)
