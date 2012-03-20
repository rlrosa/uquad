#include <uquad_aux_log.h>
#include <uquad_error_codes.h>
#include <uquad_aux_io.h>
#include <imu_comm.h>
#include <unistd.h> // For STDIN_FILENO

#define IO_TEST_HOW_TO "Wrong Arguments!\nUSAGE:\n\t./uquad_aux_io_test imu_dev some_file\nExample:\n\t./uquad_aux_io_test /dev/ttyUSB0 dummy.txt\n"
#define FREE_N_DIE_IF_ERROR(retval,msg) if(retval!=ERROR_OK){fprintf(stderr,"%s:%d:%s\n",__FILE__,__LINE__,msg);goto kill_n_close;}
int main(int argc, char *argv[]){
    int retval;
    char * device_imu;
    char * device_file;
    FILE * file;

    if(argc<3){
	fprintf(stderr,IO_TEST_HOW_TO);
	exit(1);
    }else{
	device_imu = argv[1];
	device_file = argv[2];
    }

    io_t * io = io_init();
    if(io==NULL){
	err_check(ERROR_FAIL,"io test failed");
    }

    // Lets add the IMU and some random file
    // The plan is to have the constant stream of data from the imu, a
    // sporadic data from the file, which would represent the diff in
    // update rate from the GPS and the IMU.

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Init
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // Init IMU
    imu_t * imu = imu_comm_init(device_imu);
    if(imu == NULL){
	err_check(ERROR_FAIL,"Failed to init imu");
    }

    // Init some other file
    file = fopen(device_file,"r+");
    if(file == NULL){
	err_check(ERROR_FAIL,"Failed to open file");
    }

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Register IO devices
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // imu
    int imu_fds;
    retval = imu_comm_get_fds(imu,& imu_fds);
    FREE_N_DIE_IF_ERROR(retval,"Failed to get imu fds!!");
    retval = io_add_dev(io,imu_fds);
    FREE_N_DIE_IF_ERROR(retval,"Failed to add imu to dev list");
    if(io->dev_list.dev_count != 1){
	FREE_N_DIE_IF_ERROR(retval,"Reg device counter error!"); 
    }
    // Pipe
    // To create a pipe, do:
    //     mknode my.pipe p
    // Then open my.pipe as a regular file.
    int file_fds;
    file_fds = fileno(file);
    retval = io_add_dev(io,file_fds);
    FREE_N_DIE_IF_ERROR(retval,"Failed to add file to dev list");
    if(io->dev_list.dev_count != 2){
	FREE_N_DIE_IF_ERROR(retval,"Reg device counter error!"); 
    }
    // stdin
    retval = io_add_dev(io,STDIN_FILENO);
    FREE_N_DIE_IF_ERROR(retval,"Failed to add STDIN to dev list");
    if(io->dev_list.dev_count != 3){
	FREE_N_DIE_IF_ERROR(retval,"Reg device counter error!"); 
    }

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Poll n read loop
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    uquad_bool_t read = false,write = false;
    uquad_bool_t reg_imu = true, reg_pipe = true, reg_stdin = true;
    uquad_bool_t imu_ready = false;
    imu_data_t imu_data;
    unsigned char tmp_buff[2];
    int counter = 0;
    poll_n_read:
    while(1){
	retval = io_poll(io);
	FREE_N_DIE_IF_ERROR(retval,"io_poll() error");
	// imu
	if(reg_imu){
	    retval = io_dev_ready(io,imu_fds,&read,&write);
	    FREE_N_DIE_IF_ERROR(retval,"io_dev_ready() error");
	    if(read){
		retval = imu_comm_read(imu,&imu_ready);
		if(imu_ready)
		{
		    if(retval != ERROR_OK){
			fprintf(stdout,"\nIMU missed frame?\n\n");
		    }else{
			retval = imu_comm_get_data_latest_unread(imu,&imu_data);
			if(retval != ERROR_OK)
			    fprintf(stdout,"\nIMU had no data!\n\n");
		    }
		}
	    }
	}
	// pipe
	if(reg_pipe){
	    retval = io_dev_ready(io,file_fds,&read,&write);
	    FREE_N_DIE_IF_ERROR(retval,"io_dev_ready() error");
	    if(read){
		retval = fgetc(file);
		if(retval<0)
		    fprintf(stdout,"\nEOF or error!!\n\n");
		if(retval>0){
		    fprintf(stdout,"Read data from file.\n");
		}
		if(retval==0)
		    fprintf(stdout,"File had no data!.\n");
	    }
	}
	// stdin
	if(reg_stdin){
	    retval = io_dev_ready(io,STDIN_FILENO,&read,&write);
	    FREE_N_DIE_IF_ERROR(retval,"io_dev_ready() error");
	    if(read){
		retval = fread(tmp_buff,1,1,stdin);
		if(retval<=0)
		    fprintf(stdout,"\nNo user input!!\n\n");
		else
		    // Each time user presses RET a device will be removed from
		    // the registered dev list.
		    goto rm;
	    }
	}
	fflush(stdout);
    }
	
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Remove test
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    rm:
    switch (counter){
    case 0:
	retval = io_rm_dev(io,file_fds);
	FREE_N_DIE_IF_ERROR(retval,"Failed to remove PIPE from dev list");
	fprintf(stdout,"Removed PIPE from dev list.\n");
	reg_pipe = false;
	fflush(stdout);
	break;
    case 1:
	retval = io_rm_dev(io,imu_fds);
	FREE_N_DIE_IF_ERROR(retval,"Failed to remove IMU from dev list");
	fprintf(stdout,"Removed IMU from dev list.\n");
	reg_imu = false;
	fflush(stdout);
	break;
    case 2:
	fprintf(stdout,"Removed STDIN from dev list.\n");
	retval = io_rm_dev(io,STDIN_FILENO);
	FREE_N_DIE_IF_ERROR(retval,"Failed to remove STDIN from dev list");
	reg_stdin = false;
	fflush(stdout);
	break;	
    default:
	FREE_N_DIE_IF_ERROR(ERROR_FAIL,"COUNTER out of bounds!");
	break;
    }

    if(++counter < 3)
	goto poll_n_read;

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Free everything
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    kill_n_close:
    retval = io_deinit(io);
    if(retval != ERROR_OK){
	fprintf(stderr,"Warning! Failed to free io structure!\nIgnoring error...\n");
    }
    retval = imu_comm_deinit(imu);
    if(retval != ERROR_OK){
	fprintf(stderr,"Warning! Failed to free imu structure!\nIgnoring error...\n");
    }
    return 0;
}
