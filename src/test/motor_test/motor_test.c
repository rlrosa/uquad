#include <mot_control.h>
#include <uquad_aux_math.h>
#include <sys/time.h>
#include <signal.h> // for SIGINT, SIGQUIT
#include <time.h>
#include <sys/types.h>
#include <unistd.h> // For STDIN_FILENO
#include <stdio.h>
#include <stdlib.h>

#define wait_for_enter printf("ERROR!\n") //while(fread(tmp,1,1,stdin) == 0)
static FILE *log;
static uquad_mot_t *mot;

void quit()
{
    int ret;
    err_log("Will shutdown motor driver and close logs.");
    ret =  mot_deinit(mot);
    if(log != NULL)
	fclose(log);
    exit(ret);
}

void uquad_sig_handler(int signal_num)
{
    err_log_num("Caught signal %d.\n",signal_num);
    quit();
}

#define TEST_ALL_RANGES
//#define TEST_STOP
//#define TEST_IDLE
//#define TEST_OD

void next_test(void)
{
    static int test_counter = 0;
    fprintf(log,"\n%% -- -- -- -- --\n%%Test #%d\n%% -- -- -- -- --\n",
	    test_counter++);
    sleep_ms(100);
}

int main(void){
    int retval, i;
    struct timeval tv;
    uquad_mat_t *vels = uquad_mat_alloc(MOT_C,1);
    double dtmp;

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    /// Open log file
    log = fopen("motor_test.log","w+");
    if(log == NULL)
    {
	fprintf(stderr,"Fatal error.");
	exit(1);
    }

    /// Initialize structure
    mot = mot_init();
    if(mot==NULL)
    {
	fprintf(stderr,"Fatal error.");
	exit(1);
    }

#ifdef TEST_ALL_RANGES
    /// Test all range of inputs
    next_test();
    for(i = 0; i < 3; ++i)
    {
	sleep_ms(10);
	retval = mot_set_idle(mot);
	quit_if(retval);
	usleep(MOT_UPDATE_MAX_US);    
	fprintf(stdout,"\n\n-- -- -- -- --\nLoop #%d\n-- -- -- -- --\n\n",i);
	for (dtmp = 110.0; dtmp < 350.0; dtmp++)
	{
	    vels->m_full[0] = dtmp;
	    vels->m_full[1] = dtmp + 0.2;
	    vels->m_full[2] = dtmp + 3.69;
	    vels->m_full[3] = dtmp + 1.0;
	    retval = mot_set_vel_rads(mot,vels);
	    gettimeofday(&tv,NULL);
	    fprintf(log,"%d\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%d\n",
		    retval,
		    vels->m_full[0],
		    vels->m_full[1],
		    vels->m_full[2],
		    vels->m_full[3],
		    (int)tv.tv_usec);
	    usleep(MOT_UPDATE_MAX_US);
	}
	retval = mot_stop(mot);
	quit_if(retval);
	usleep(MOT_UPDATE_MAX_US);	    
    }
#endif


#ifdef TEST_STOP
    /// Test stop
    next_test();
    retval = mot_stop(mot);
    gettimeofday(&tv,NULL);
    fprintf(log,"%d\t%d\n", retval, (int)tv.tv_usec);
#endif

#ifdef TEST_IDLE
    /// Test idle
    next_test();
    retval = mot_set_idle(mot);
    gettimeofday(&tv,NULL);
    fprintf(log,"%d\t%d\n", retval, (int)tv.tv_usec);
#endif

#ifdef TEST_OD
    /// Test overdose - should only get first value
    next_test();
    for (dtmp = 300.0, i = 0; i < 100; ++i)
    {
	vels->m_full[0] = dtmp;
	vels->m_full[1] = dtmp - 1.0;
	vels->m_full[2] = dtmp + 1.0;
	vels->m_full[3] = dtmp + 1.0;

	retval = mot_set_vel_rads(mot,vels);
	gettimeofday(&tv,NULL);
	fprintf(log,"%d\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%d\n",
		retval,
		vels->m_full[0],
		vels->m_full[1],
		vels->m_full[2],
		vels->m_full[3],
		(int)tv.tv_usec);
	if(i < 50)
	    usleep(MOT_UPDATE_MAX_US);
	else
	    usleep(MOT_UPDATE_MAX_US/2);
    }
#endif

    /// Test Deinit
    next_test();
    retval = mot_deinit(mot);
    uquad_mat_free(vels);
    gettimeofday(&tv,NULL);
    fprintf(log,"%d\t%d\n", retval, (int)tv.tv_usec);

    printf("\n\n%%All tests completed!\n");

    return 0;
}
    
