/**
 * uquad_aux_time_test: test program for uquad_time
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
 * @file   uquad_aux_time_test.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief test for uquad_time
 */
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>
#include <stdio.h>

#define UQUAD_AUX_TIME_TEST_COUNT 3

int main(void){
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
