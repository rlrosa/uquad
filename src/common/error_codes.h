#define ERROR_OK 0
#define ERROR_FAIL -1
#define ERROR_READ_TIMEOUT -2
#define ERROR_READ_SYNC -3
#define ERROR_OPEN -4
#define ERROR_CLOSE -5
#define ERROR_MALLOC -6
#define ERROR_IMU_AVG_NOT_ENOUGH -7
#define ERROR_IO -8
#define ERROR_WRITE -9
#define ERROR_INVALID_ARG -10
#define ERROR_NULL_POINTER -11

#define err_propagate(retval) if(retval!=ERROR_OK){fprintf(stderr,"backtrace:%s:%d\n",__FILE__,__LINE__);return retval;}
#define err_check(retval,msg) if(retval!=ERROR_OK){fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,msg);return retval;}

/// No functions
