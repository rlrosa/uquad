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
 * @file   uquad_aux_io.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief lib to handle a list of IO devices, and read/write from them in a non blocking manner.
 *
 */
#include <uquad_aux_io.h>

io_t * io_init(void){
    io_t * io;
    io = (io_t *)malloc(sizeof(io_t));
    if(io == NULL){
	fprintf(stderr,"Failed to allocate mem. \n");
	return io;
    }
    io->dev_list.first = NULL;
    io->dev_list.dev_count = 0;
    return io;
}

/// -- -- -- -- -- -- -- -- -- -- -- --
/// Device list handling
/// -- -- -- -- -- -- -- -- -- -- -- --

uquad_bool_t io_dev_list_empty(io_t * io){
    return (io->dev_list.first == NULL)?true:false;
}

static int io_get_dev_list(io_t * io, io_dev_t ** dev){
    if(io_dev_list_empty(io)){
	err_check(ERROR_FAIL,"Cannot get IO list! List is empty!");
    }
    *dev = io->dev_list.first;
    return ERROR_OK;
}

static int io_get_last_dev(io_t * io, io_dev_t ** last_dev){
    if(io_dev_list_empty(io)){
	*last_dev = NULL;
	return ERROR_OK;
    }
    io_dev_t * dev = io->dev_list.first;
    while(dev->next != NULL)
	dev = dev->next;
    *last_dev = dev;
    return ERROR_OK;
}

static int io_dev_init(io_dev_t * dev, int fd){
    if(dev == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot init dev, dev is null.");
    }
    dev->next = NULL;
    dev->ready_read = false;
    dev->ready_write = false;
    dev->fd = fd;
    return ERROR_OK;
}

int io_add_dev(io_t * io, int fd){
    io_dev_t * new_io_dev;
    int retval;
    new_io_dev = (io_dev_t *)malloc(sizeof(io_dev_t));
    if(new_io_dev == NULL){
	err_check(ERROR_MALLOC,"Failed to allocate mem.");
    }
    retval = io_dev_init(new_io_dev,fd);
    err_propagate(retval);
    io_dev_t * last_dev;
    retval = io_get_last_dev(io,&last_dev);
    if(last_dev == NULL){
	// start new list
	io->dev_list.first = new_io_dev;
    }else{
	last_dev->next = new_io_dev;
    }
    ++io->dev_list.dev_count;
    return ERROR_OK;
}

int io_rm_dev(io_t * io, int fd){
    if(io_dev_list_empty(io)){
	err_check(ERROR_FAIL,"Device not found (empty list)");
    }
    io_dev_t * dev = io->dev_list.first;
    io_dev_t * dev_prev = NULL;
    do{
	if(dev->fd == fd)
	    break;
	dev_prev = dev;
	dev = dev->next;
    }while(dev != NULL);

    if(dev == NULL){
	err_check(ERROR_FAIL,"Device not found");
    }

    if(dev_prev == NULL){
	// Remove the first element of the list
	io->dev_list.first = dev->next;
    }else{
	// skip the one that will be deleted, but keep the tail
	dev_prev->next = dev->next;
    }
    free(dev);
    --io->dev_list.dev_count;

    return ERROR_OK;
}

/// -- -- -- -- -- -- -- -- -- -- -- --
/// Polling
/// -- -- -- -- -- -- -- -- -- -- -- --

int io_poll(io_t * io){
    io_dev_t * dev;
    int retval;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    if(io_dev_list_empty(io)){
	return ERROR_OK;
    }
    retval = io_get_dev_list(io,&dev);
    err_propagate(retval);
    fd_set rfds,wfds;
    int max_fd = 0;
    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    while(dev != NULL){
	FD_SET(dev->fd,&rfds);
	FD_SET(dev->fd,&wfds);
	max_fd = (max_fd > dev->fd)?max_fd:dev->fd;
	dev = dev->next;
    }
    retval = select(max_fd + 1,&rfds,&wfds,NULL,&tv);
    if(retval < 0){
	err_check(ERROR_IO,"select() failed!");
    }
    
    // Now go check who is ready
    retval = io_get_dev_list(io,&dev);
    err_propagate(retval);
    while(dev != NULL){
	// check if ready to read
	dev->ready_read = (FD_ISSET(dev->fd,&rfds))?true:false;
	// check if ready to write
	dev->ready_write = (FD_ISSET(dev->fd,&wfds))?true:false;
	// keep checking
	dev = dev->next;
    }
    return ERROR_OK;
}

int io_get_dev(io_t * io, int fd, io_dev_t ** dev){
    int retval;
    io_dev_t * dev_curr = NULL;
    retval = io_get_dev_list(io,&dev_curr);
    err_propagate(retval);
    while(dev_curr != NULL){
	if(fd == dev_curr->fd){
	    *dev = dev_curr;
	    return ERROR_OK;
	}
	dev_curr = dev_curr->next;
    }
    err_check(ERROR_IO_DEV_NOT_FOUND,"The fd supplied is not registered...");
}			   

int io_dev_ready(io_t * io, int fd, uquad_bool_t * read, uquad_bool_t * write){
    int retval;
    io_dev_t * dev;
    retval = io_get_dev(io,fd,&dev);
    err_propagate(retval);
    if(read != NULL)
	*read = dev->ready_read;
    if(write != NULL)
	*write = dev->ready_write;
    return ERROR_OK;
}

int io_deinit(io_t * io){
    int retval;
    io_dev_t * dev, * dev_next;
    if((io != NULL) && !io_dev_list_empty(io)){
	// Free all registered devices
	retval = io_get_dev_list(io,&dev);
	err_propagate(retval);
	while(dev != NULL){
	    dev_next = dev->next;
	    free(dev);
	    dev = dev_next;
	}
	free(io);
    }
    return ERROR_OK;
}

int check_io_locks(int fd, FILE *device, uquad_bool_t *read_ok, uquad_bool_t *write_ok){
    fd_set rfds,wfds;
    struct timeval tv;
    int retval = ERROR_OK;
    if(fd < 0)
    {
	fd = fileno(device);
	if(fd < 0)
	{
	    err_log_stderr("Failed to get fd!");
	    err_propagate(ERROR_IO);
	}
    }
    FD_ZERO(&rfds);
    FD_SET(fd,&rfds);
    FD_ZERO(&wfds);
    FD_SET(fd,&wfds);
    // Set time waiting time to zero
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    // Check if we read/write without locking
    retval = select(fd+1,				\
		    (read_ok != NULL)?&rfds:NULL,	\
		    (write_ok != NULL)?&wfds:NULL,	\
		    NULL,&tv);
    if(read_ok != NULL){
	*read_ok = ((retval > 0) && FD_ISSET(fd,&rfds)) ? true:false;
    }
    if(write_ok != NULL)
	*write_ok = ((retval >0) && FD_ISSET(fd,&wfds)) ? true:false;
    if (retval < 0)
    {
	err_log_stderr("select() failed!");
	err_propagate(ERROR_IO);
    }
    // else: retval == 0 <--> No data available
    return ERROR_OK;
}
