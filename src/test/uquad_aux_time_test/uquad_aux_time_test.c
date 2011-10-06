#include <uquad_aux_time.h>
#include <error_codes.h>
#include <stdio.h>

#define UQUAD_AUX_TIME_TEST_COUNT 3

int main(int argc, char *argv[]){
    int retval,test_counter = 0;
    struct timeval res,t1,t2;
    t1.tv_sec = 150;
    t1.tv_usec = 250;
    t2.tv_sec = 100;
    t2.tv_usec = 400;
    // Test 1
    // t1>t2
    retval = uquad_timeval_substract(&res,t1,t2);
    if(retval <= 0){//should be positive
	err_check(ERROR_FAIL,"uquad_timeval_substract failed!");
    }else{
	++test_counter;
	printf("Test %d of %d: success!\n",test_counter,UQUAD_AUX_TIME_TEST_COUNT);
    }
    // Test 2
    // t1>t2
    retval = uquad_timeval_substract(&res,t2,t1);// (swapped order)
    if(retval >= 0){//should be negative
	err_check(ERROR_FAIL,"uquad_timeval_substract failed!");
    }else{
	++test_counter;
	printf("Test %d of %d: success!\n",test_counter,UQUAD_AUX_TIME_TEST_COUNT);
    }
    // Test 3
    // t1==t1
    retval = uquad_timeval_substract(&res,t1,t1);
    if(retval != 0){//should NOT be negative
	err_check(ERROR_FAIL,"uquad_timeval_substract failed!");
    }else{
	++test_counter;
	printf("Test %d of %d: success!\n",test_counter,UQUAD_AUX_TIME_TEST_COUNT);
    }
    printf("Tests succedded!\n");
    return ERROR_OK;
}
