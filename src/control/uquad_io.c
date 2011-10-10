#include <uquad_io.h>

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

static int io_get_dev_count(io_t * io, int * dev_count){
    *dev_count = io->io_dev_list.dev_count;
    return ERROR_OK;
}

uquad_bool_t io_dev_list_empty(io_t * io){
    return (io->dev_list.first == NULL)?true:false;
}

static int io_get_last_dev(io_t * io, io_dev_t * & last_dev){
    int dev_count_local = 0;
    if(io_dev_list_empty(io)){
	last_dev = NULL;
	return ERROR_OK;
    }
    io_dev_t * dev = io->dev_list.first;
    while(dev->next != NULL)
	dev = dev->next;
    last_dev = dev;
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
	err_check(ERROR_MALOC,"Failed to allocate mem.");
    }
    retval = io_dev_init(new_io_dev,fd);
    err_propagate(retval);
    io_dev_t * last_dev;
    retval = io_get_last_dev(io,last_dev);
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
	if(dev->fd = fd)
	    break;
	dev_prev = dev;
	dev = dev->next;
    }while(dev != NULL);

    if(dev == NULL){
	err_check(ERROR_FAIL,"Device not found");
    }

    if(dev_prev == NULL){
	// there was only 1 element, and it will be removed
	io->dev_list.first = NULL;
    }else{
	// skip the one that will be deleted, but keep the tail
	dev_prev->next = dev->next;
    }
    free(dev);
    --io->dev_list.dev_count;

    return ERROR_OK;
}

int io_poll(io_t * io){
    //TODO
    err_check(ERROR_FAIL,"Not implemented!");
}

int io_dev_has_data(io_t * io, int fd){
    //TODO
    err_check(ERROR_FAIL,"Not implemented!");
}
