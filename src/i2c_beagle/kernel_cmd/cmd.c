#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include "/usr/include/linux/i2c.h"
//#include </usr/include/linux/i2c-dev.h.kernel>
//#include <linux/i2c.h>
//#include <include/linux/i2c.h>
//#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>

#include <linux/types.h>
//#include <sys/stat.h>
//#include <stdint.h>
//#include <linux/fcntl.h>
#include <linux/time.h>

// kernel queues
#include <linux/ipc.h> // for IPC_NOWAIT
#include <linux/msg.h>


MODULE_LICENSE("GPL");

#define MS_TO_NS(x)	(x * 1E6L)
#define DELAY_MS 2L
#define RUNS 200

#define FOO_REG_GENERIC   0
#define FOO_GENERIC_VALUE 0
#define FOO_EXTENT        0
#define FOO_VERSION       "0.0"
#define FOO_DATE          "0"

#define MAX_SPEED                 220 // i2c
#define MIN_SPEED                 45  // i2c
#define IDLE_SPEED                50  // i2c

#define DEBUG                     0
#define LOG_VELS                  (1 && DEBUG)

#define PRINT_COUNT               500
#define MOT_COUNT                 4
#define UQUAD_MOT_I2C_REG_ENABLE  0xA0
#define UQUAD_MOT_ENABLE_DAT      0x00
#define UQUAD_MOT_I2C_REG_DISABLE 0xA1
#define UQUAD_MOT_DISABLE_DAT     0x00
#define UQUAD_MOT_I2C_REG_VEL     0xA2
#define MOT_SELECTED              1
#define MOT_NOT_SELECTED          0

/// Startup parameters
#define RAMP_START                30
#define RAMP_END                  50
#define START_JITTER              25
#define STARTUP_SWAP              0

#define OK                        0
#define NOT_OK                    -1

#define UQUAD_STARTUP_RETRIES     100
#define UQUAD_STOP_RETRIES        1000

#define backtrace()     printk("%s:%d\n",__FUNCTION__,__LINE__)
#define log_to_err(msg) printk("%s: %s:%d\n",msg,__FUNCTION__,__LINE__)

typedef enum {RUNNING,STOPPED}mot_state_t;

int foo_attach_adapter(struct i2c_adapter *adapter)
{
    return OK;
}
int foo_detach_client(struct i2c_client *client)
{
    int err;

    /* Try to detach the client from i2c space */
    if ((err = i2c_detach_client(client))) {
	printk("foo.o: Client deregistration failed, client not detached.\n");
	return err;
    }

    kfree(client->data);
    return 0;
}

static struct i2c_driver foo_driver = {
    //	.name		= "Foo version 2.3 driver",
    //	.id		= I2C_DRIVERID_FOO, /* from i2c-id.h, optional */
    //	.flags		= I2C_DF_NOTIFY,
    .attach_adapter	= foo_attach_adapter,
    .detach_client	= foo_detach_client,
    .command	= foo_command, /* may be NULL */
    .inc_use	= foo_inc_use, /* May be NULL */
    .dec_use	= foo_dec_use, /* May be NULL */
}

struct foo_data {
    struct i2c_client client;
    struct semaphore lock; /* For ISA access in `sensors' drivers. */
    int sysctl_id;         /* To keep the /proc directory entry for
                              `sensors' drivers. */
    //    enum chips type;       /* To keep the chips type for `sensors' drivers. */

    /* Because the i2c bus is slow, it is often useful to cache the read
       information of a chip for some time (for example, 1 or 2 seconds).
       It depends of course on the device whether this is really worthwhile
       or even sensible. */
    struct semaphore update_lock; /* When we are reading lots of information,
                                     another process should not update the
                                     below information */
    char valid;                   /* != 0 if the following fields are valid. */
    unsigned long last_updated;   /* In jiffies */
    /* Add the read information here too */
};

/// Global vars
static struct hrtimer hr_timer;
static ktime_t ktime;
static mot_state_t  m_status = STOPPED;

/// Forwards defs
int uquad_mot_i2c_addr_open(int i2c_dev, int addr);
int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value);
int uquad_mot_i2c_send(int i2c_dev, int i2c_addr, __u8 reg, __u8 val);

int uquad_mot_enable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_enable_all(int i2c_dev);
int uquad_mot_disable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_disable_all(int i2c_dev);
int uquad_mot_set_speed(int i2c_dev, int mot_i2c_addr, __u8 val);
int uquad_mot_set_speed_all(int i2c_dev, __u8 *v, int swap_order);


// Global vars
/* static int msqid; // Set by mot_control.h */
/* const static key_t key_s = 169; // must match MOT_SERVER_KEY (in mot_control.h) */
/* const static key_t key_c = 170; // must match MOT_DRIVER_KEY (in mot_control.h) */
/* static message_buf_t rbuf; */
static int i2c_file = -1;
static int mot_i2c_addr[MOT_COUNT] = {0x69,
				      0x6a,
				      0x6b,
				      0x68};
static unsigned short mot_selected[MOT_COUNT] = {MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED};

//static __u8 vels[MOT_COUNT] = {0,0,0,0};
static __u8 vels[MOT_COUNT] = {IDLE_SPEED,
			       IDLE_SPEED,
			       IDLE_SPEED,
			       IDLE_SPEED};

#if LOG_VELS
static FILE *log_rx;
#endif

int uquad_mot_i2c_addr_open(int i2c_dev, int addr){
    /* if (ioctl(i2c_dev,I2C_SLAVE,addr) < 0) */
    /* { */
    /* 	/\* ERROR HANDLING; you can check errno to see what went wrong *\/ */
    /* 	printk("ERROR! %s failed to write to 0x%02X...\n",__FUNCTION__,addr); */
    /* 	return NOT_OK; */
    /* } */
    return OK;
}

int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value){
    /* int ret; */
    /* if(i2c_smbus_write_byte_data(i2c_dev,reg,value) < 0) */
    /* { */
    /* 	/\* ERROR HANDLING: i2c transaction failed *\/ */
    /* 	printk(_ERR,"Failed to send value %d\tto 0x%02X\n.", (int)value,(int)reg); */
    /* 	return NOT_OK; */
    /* } */
    return OK;
}

int uquad_mot_i2c_send(int i2c_dev, int i2c_addr, __u8 reg, __u8 val){
    if(uquad_mot_i2c_addr_open(i2c_dev, i2c_addr) < 0)
    {
	backtrace();
	return NOT_OK;
    }
    if(uquad_mot_i2c_send_byte(i2c_dev,reg,val) < 0)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

int uquad_mot_enable(int i2c_dev, int mot_i2c_addr){
    return uquad_mot_i2c_send(
				 i2c_dev,
				 mot_i2c_addr,
				 UQUAD_MOT_I2C_REG_ENABLE,
				 UQUAD_MOT_ENABLE_DAT);
}

int uquad_mot_enable_all(int i2c_dev){
    int i;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(!mot_selected[i])
	   continue;
	if(uquad_mot_enable(i2c_dev,mot_i2c_addr[i]))
	{
	    backtrace();
	    break;
	}
    }
    if(i<MOT_COUNT)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

int uquad_mot_disable(int i2c_dev, int mot_i2c_addr){
    return uquad_mot_i2c_send(
				 i2c_dev,
				 mot_i2c_addr,
				 UQUAD_MOT_I2C_REG_DISABLE,
				 UQUAD_MOT_DISABLE_DAT);
}

int uquad_mot_disable_all(int i2c_dev){
    int i;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(!mot_selected[i])
	    continue;
	if(uquad_mot_disable(i2c_dev,mot_i2c_addr[i]))
	{
	    backtrace();
	    break;
	}
    }
    if(i<MOT_COUNT)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

int uquad_mot_set_speed(int i2c_dev, int mot_i2c_addr, __u8 val){
    return uquad_mot_i2c_send(
				 i2c_dev,
				 mot_i2c_addr,
				 UQUAD_MOT_I2C_REG_VEL,
				 val);
}

int uquad_mot_set_speed_all(int i2c_dev, __u8 *v, int swap_order){
    int i, index;
    __u8 u8tmp = 0;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(!mot_selected[i])
	    continue;
	index = swap_order?(3+i)%4:i;
	u8tmp = (v==NULL)?0:v[index];
	if(uquad_mot_set_speed(i2c_dev,
			       mot_i2c_addr[index],
			       u8tmp))
	{
	    backtrace();
	    break;
	}
    }
    if(i<MOT_COUNT)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

#define STARTUPS 1 // send startup command repeatedly
int uquad_mot_startup_all(int i2c_file){
    int i, j, watchdog = 0, enable_counts = 0;
    __u8 tmp_vel[MOT_COUNT]; __u8 ramp = 0;
#if STARTUP_SWAP
    if(uquad_mot_set_speed_all(i2c_file, NULL,1) < 0)
    {
	backtrace();
	return NOT_OK;
    }
#endif
    while(uquad_mot_enable_all(i2c_file) < 0 || ++enable_counts < STARTUPS)
    {
	udelay(400);
	// wait for start
	if(++watchdog == UQUAD_STARTUP_RETRIES)
	{
	    backtrace();
	    return NOT_OK;
	}
    }
    mdelay(420L);
    for(ramp = RAMP_START; ramp < RAMP_END; ramp++)
    {
	for(j = 0; j < 2*ramp; ++j)
	{
	    if(j!=0)
		udelay(1235L);
	    for(i = 0; i < MOT_COUNT; ++i)
		tmp_vel[i] = (!(j%7))?
		    ramp-START_JITTER:
		    ramp+START_JITTER;
	    uquad_mot_set_speed_all(i2c_file, tmp_vel,0);
	}
    }
    return OK;
}

int uquad_mot_stop_all(int i2c_file, __u8 *v){
    int watchdog = 0;
    if(uquad_mot_set_speed_all(i2c_file, v,1) < 0)
    {
	backtrace();
	return NOT_OK;
    }	
    while(uquad_mot_disable_all(i2c_file) < 0)
    {
	// wait for stop
	if(++watchdog == UQUAD_STOP_RETRIES)
	{
	    backtrace();
	    return NOT_OK;
	}
    }
    return OK;
}

#if LOG_VELS
unsigned long rx_counter = 0;
static inline void log_vels(void)
{
    gettimeofday(&timestamp,NULL);
    fprintf(log_rx,"%d\t%d\t%d\t%d\t%d\t%lu\n",
	    (int)vels[0],
	    (int)vels[1],
	    (int)vels[2],
	    (int)vels[3],
	    (int)timestamp.tv_usec,
	    rx_counter++);
}
#endif

int mot_update(void)
{
    int
	ret = OK,
	i;
    // startup
    if( (m_status == STOPPED) && (
				  (vels[0] > MIN_SPEED)&&
				  (vels[1] > MIN_SPEED)&&
				  (vels[2] > MIN_SPEED)&&
				  (vels[3] > MIN_SPEED)))
    {
	printk("Will startup motors...\n");
	ret = uquad_mot_startup_all(i2c_file);
	if(ret != OK)
	{
	    printk("FAILED to startup motors...\n");
	}
	else
	{
	    printk("Startup was successfull!...\n");
	    m_status = RUNNING;
	}
    }
    else
    {
	// stop
	if ((m_status == RUNNING) && (
				      (vels[0] == 0) ||
				      (vels[1] == 0) ||
				      (vels[2] == 0) ||
				      (vels[3] == 0)
				      )
	    )
	{
	    printk("Will stop motors...\n");
	    ret = uquad_mot_stop_all(i2c_file, vels);
	    if(ret != OK)
	    {
		printk("FAILED to stop motors...");
	    }
	    else
	    {
		printk("Stopping was successfull!...");
		m_status = STOPPED;
	    }
	}
	else
	{
	    // normal call
	    for(i = 0; i < MOT_COUNT; ++i)
	    {
		ret = uquad_mot_set_speed(i2c_file,
					  mot_i2c_addr[i],
					  vels[i]);
		if(ret != OK)
		    break;
	    }
	}
    }
    // NO ACK!
    return ret;
}

int foo_detect_client(struct i2c_adapter *adapter, int address,
		      unsigned short flags, int kind)
{
    int err = 0;
    struct i2c_client *new_client;
    struct foo_data *data;
    const char *client_name = ""; /* For non-`sensors' drivers, put the real
                                     name here! */

    /* Let's see whether this adapter can support what we need.
       Please substitute the things you need here!
       For `sensors' drivers, add `! is_isa &&' to the if statement */
    if (!i2c_check_functionality(adapter,I2C_FUNC_SMBUS_WORD_DATA |
				 I2C_FUNC_SMBUS_WRITE_BYTE))
	goto ERROR0;

    /* OK. For now, we presume we have a valid client. We now create the
       client structure, even though we cannot fill it completely yet.
       But it allows us to access several i2c functions safely */

    if (!(data = kzalloc(sizeof(struct foo_data), GFP_KERNEL))) {
	err = -ENOMEM;
	goto ERROR0;
    }

    new_client = &data->client;
    new_client->addr = address;
    //    new_client->data = data;
    new_client->adapter = adapter;
    new_client->driver = &foo_driver;
    new_client->flags = 0;

    /* Now, we do the remaining detection. If no `force' parameter is used. */

    /* First, the generic detection (if any), that is skipped if any force
       parameter was used. */
    if (kind < 0) {
      /* The below is of course bogus */
      if (foo_read(new_client,FOO_REG_GENERIC) != FOO_GENERIC_VALUE)
         goto ERROR1;
    }

    /* Fill in the remaining client fields. */
    strcpy(new_client->name,client_name);

    data->valid = 0; /* Only if you use this field */
    init_MUTEX(&data->update_lock); /* Only if you use this field */

    /* Any other initializations in data must be done here too. */

    /* Tell the i2c layer a new client has arrived */
    if ((err = i2c_attach_client(new_client)))
      goto ERROR3;


    /* This function can write default values to the client registers, if
       needed. */
    foo_init_client(new_client);
    return 0;

    /* OK, this is not exactly good programming practice, usually. But it is
       very code-efficient in this case. */

    ERROR3:
    /* SENSORS ONLY START */
      if (is_isa)
        release_region(address,FOO_EXTENT);
    /* SENSORS ONLY END */
    ERROR1:
      kfree(data);
    ERROR0:
      return err;
}

enum hrtimer_restart my_hrtimer_callback( struct hrtimer *timer )
{
    int
	ret,
	overruns;
    overruns = hrtimer_forward_now(timer, ktime);
    if(overruns > 1)
	printk( "overruns! (%d)\n", overruns );
    printk( "my_hrtimer_callback called (%ld).\n", jiffies );

    ret = mot_update();
    if(ret != OK)
	printk("Failed to update motors!");

    return HRTIMER_RESTART; 
}

static int foo_initialized = 0;

int __init foo_cleanup(void)
{
    int res;
    if (foo_initialized == 1) {
	if ((res = i2c_del_driver(&foo_driver))) {
	    printk("foo: Driver registration failed, module not removed.\n");
	    return res;
	}
	foo_initialized --;
    }
    return 0;
}

int __init foo_init(void)
{
    int res;
    printk("foo version %s (%s)\n",FOO_VERSION,FOO_DATE);

    if ((res = i2c_add_driver(&foo_driver))) {
	printk("foo: Driver registration failed, module not inserted.\n");
	foo_cleanup();
	return res;
    }
    foo_initialized ++;
    return 0;
}

int init_module( void )
{
    //  char *filename = "/dev/i2c-2";
    int i, ret = OK;

    printk("Installing cmd...\n");

    for(i=0;i<MOT_COUNT;++i)
	mot_selected[i] = MOT_SELECTED;

    // Open i2c
    ret = foo_init();
    if(ret < 0)
    {
	printk("Failed to init i2c!");
	return ret;
    }
    /* if ((i2c_file = open(filename,O_RDWR)) < 0) */
    /* { */
    /*     printk("ERROR! Failed to open %s!\nAborting...\n",filename); */
    /*     printk("errno info:\t %s\n",strerror(errno)); */
    /*     return NOT_OK; */
    /* } */

    m_status = STOPPED;
    ret = uquad_mot_enable_all(i2c_file);
    if(ret != OK)
	return ret;

    ktime = ktime_set( 0, MS_TO_NS(DELAY_MS) );

    hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
  
    hr_timer.function = &my_hrtimer_callback;

    printk( "Starting timer to fire in %ldms (%ld)\n", DELAY_MS, jiffies );

    hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );

    return 0;
}

void cleanup_module( void )
{
    int ret = OK, watchdog = 20;

    (void) hrtimer_cancel( &hr_timer );
    // ignore answer

    if( i2c_file>= 0 )
    {
	printk("Shutting down motors...\n");
	while(1)
	{
	    ret = uquad_mot_disable_all(i2c_file);
	    if(ret != OK)
	    {
		if(watchdog-- < 0)
		{
		    printk("cmd cleanup Timed out!");
		    break;
		}
		printk("Failed to shutdown motors!... Retries left:%d...\n",watchdog);
	    }
	    else
	    {
	      printk("Motors successfully stoped!\n");
	      break;
	    }
	}
    }
}
