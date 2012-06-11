/**
 * uquad_io: Library to handle a list of io devices, and read/write from them in a non blocking manner.
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   uquad_aux_io.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief lib to handle a list of IO devices, and read/write from them in a non blocking manner.
 *
 * Examples:
 *   - src/main/main.c
 *   - test/uquad_aux_io_test/uquad_aux_io_test.c
 */
#ifndef UQUAD_AUX_IO_H
#define UQUAD_AUX_IO_H

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <uquad_error_codes.h>
#include <uquad_types.h>

typedef struct io_dev{
    int fd;
    uquad_bool_t ready_read;
    uquad_bool_t ready_write;
    struct io_dev * next;
} io_dev_t;

typedef struct io_dev_list{
    int dev_count;
    io_dev_t * first;
} io_dev_list_t;

typedef struct io{
    io_dev_list_t dev_list;
} io_t;

/**
 * Initializes an empty io structure.
 * Failure is reported by returning a NULL pointer.
 *
 *
 * @return structure created 
 */
io_t * io_init(void);

/**
 * Unregisters all registered devices and frees all mem.
 *
 * @param io 
 *
 * @return error code
 */
int io_deinit(io_t * io);

/// -- -- -- -- -- -- -- -- -- -- -- --
/// Device list handling
/// -- -- -- -- -- -- -- -- -- -- -- --

/**
 * Registers a device given by FD.
 * All registered devices will be checked for read/write when polling.
 *
 * @param io 
 * @param fd device to register
 *
 * @return error code
 */
int io_add_dev(io_t * io, int fd);

/**
 * Removes the device given by FD from the registered devices.
 * Device FD will no longer be checked for read/write.
 *
 * @param io 
 * @param fd device to remove
 *
 * @return error code
 */
int io_rm_dev(io_t * io, int fd);

/// -- -- -- -- -- -- -- -- -- -- -- --
/// Polling
/// -- -- -- -- -- -- -- -- -- -- -- --

/**
 * Checks registered devices for read/write availability.
 * This function must be called before io_dev_ready, since this function
 * is responsible for updating the read/write availability.
 *
 * @param io 
 *
 * @return error code
 */
int io_poll(io_t * io);

/**
 * Get the device identified by FD
 *
 * @param io 
 * @param dev if successful, device identified by FD is returned here.
 *
 * @return error code
 */
int io_get_dev(io_t * io, int fd, io_dev_t ** dev);

/**
 * Checks if device identified by FD is ready to read/write.
 * If READ==NULL, then read check is not performed.
 * If WRITE==NULL, then write check is not performed.
 * This function must be called after calling io_poll, since io_poll is responsible
 * for updating the status.
 *
 * @param io 
 * @param fd device to check
 * @param read if successful, the value of *read represents the read status of FD 
 * @param write if successful, the value of *write represents the read status of FD 
 *
 * @return error code
 */
int io_dev_ready(io_t * io, int fd, uquad_bool_t * read, uquad_bool_t * write);

/**
 *Checks if reading/writing will block.
 *Writing should not be a problem, hw buffers should handle it.
 *If attempting to read and there is no data available, we do not want to
 *lock up the sys, that is the purpose of 'select'.
 *
 *Will take either a fd as argument. If fd is negative, then will get
 *the fd from the string device.
 *
 *@param fd file descriptor of device attempting to read from, or < 0 if 
 *device should be used.
 *@param device attemping to read or write to.
 *@param check_read if true then checks if reading locks, if false check writing.
 *@param ready answer returned here
 *
 *@return error code
 */
int check_io_locks(int fd, FILE *device, uquad_bool_t *read_ok, uquad_bool_t *write_ok);

#endif
