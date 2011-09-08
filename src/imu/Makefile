CC=gcc
CFLAGS=-Wall -g
SOURCES=imu_comm_test.c imu_comm.h imu_comm.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=imu_comm_test

all: $(SOURCES)
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(SOURCES)
clean:
	rm *.o $(EXECUTABLE)

