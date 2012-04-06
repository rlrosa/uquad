EXECUTABLE=cmd
CC=gcc
CFLAGS=-O
SOURCES=cmd_motores.c
all:
	$(CC) $(CFLAGS) cmd_motores.c -o $(EXECUTABLE)
debug:
	$(CC) -g $(SOURCES) -o $(EXECUTABLE)_debug
pc_test:
	$(CC) -g -DPC_TEST $(SOURCES) -o $(EXECUTABLE)pc_test
stdin:
	$(CC) -g -DCHECK_STDIN $(SOURCES) -o $(EXECUTABLE)stdin
clean:
	rm $(EXECUTABLE)
