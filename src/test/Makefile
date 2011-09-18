CC=gcc
CFLAGS=-Wall -g
SOURCES=imu_comm_test.c  ../imu/imu_comm.h ../imu/imu_comm.c
INCLUDE_DIR=$(UQUAD_SRC)/imu/
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=imu_comm_test

all:
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(SOURCES) -I$(INCLUDE_DIR)
clean:
	rm *.o $(EXECUTABLE)

