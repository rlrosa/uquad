EXECUTABLE=cmd
CC=gcc
CFLAGS=-g
all:pipe queue
queue:
	$(CC) $(CFLAGS) message_send.c -o q_send
	$(CC) $(CFLAGS) message_rec.c -o q_rec
pipe:
	$(CC) $(CFLAGS) pipe_send.c -o p_send
	$(CC) $(CFLAGS) pipe_rec.c -o p_rec
clean:
	rm *.o
	rm q_send q_rec
	rm p_send p_rec
