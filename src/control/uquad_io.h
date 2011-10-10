#ifndef IMU_COMM_H
#define IMU_COMM_H

#include <uquad_error_codes.h>
#include <uquad_types.h>

struct io_dev{
    int fd;
    uquad_bool_t ready_read;
    uquad_bool_t ready_write;
    struct io_dev * next;
}
typedef struct io_dev io_dev_t;

struct io_dev_list{
    int dev_count;
    io_dev_t * first;
}
typedef struct io_dev_list io_dev_list_t;

struct io{
    io_dev_list_t dev_list;
}
typedef struct io io_t;

io * io_init(void);
int io_add_dev(io * io, int fd);
int io_rm_dev(io * io, int fd);
int io_poll(io * io);
int io_dev_has_data(io * io, int fd);

#endif
