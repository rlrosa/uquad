EXECUTABLE=cmd
CC=gcc
CFLAGS=-Wall
LDFLAGS=-lm
SOURCES=cmd_motores.c
all:
	$(CC) -O3 $(CFLAGS) $(LDFLAGS) cmd_motores.c -o $(EXECUTABLE)
debug:
	$(CC) -g $(SOURCES) -o $(EXECUTABLE)_debug
pc_test:
	$(CC) -g -DPC_TEST $(LDFLAGS) $(CFLAGS) $(SOURCES) -o $(EXECUTABLE)pc_test
stdin:
	$(CC) -DCHECK_STDIN $(LDFLAGS) $(CFLAGS) $(SOURCES) -o $(EXECUTABLE)stdin
clean:
	rm $(EXECUTABLE)
