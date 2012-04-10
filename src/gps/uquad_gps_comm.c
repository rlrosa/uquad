#include <uquad_gps_comm.h>
#include <stdlib.h>
#include <gpsd.h>
#include <math.h>
#include <uquad_error_codes.h>

#define GPS_COMM_STREAM_FLAGS_ENA WATCH_ENABLE | WATCH_JSON
#define GPS_COMM_STREAM_FLAGS_DIS WATCH_DISABLE

static uquad_mat_t *m1x3 = NULL;

gps_t *  gps_comm_init(void){
    gps_t * gps;
    int retval;
    gps = (gps_t *)malloc(sizeof(gps_t));
    cleanup_if_null(gps);

    gps->gpsd = (gpsd_t *)malloc(sizeof(gpsd_t));
    cleanup_if_null(gps->gpsd);

    gps->pos = uquad_mat_alloc(3,1);
    cleanup_if_null(gps->gpsd);
    gps->pos_ep = uquad_mat_alloc(3,1);
    cleanup_if_null(gps->gpsd);

    gps->vel = uquad_mat_alloc(3,1);
    cleanup_if_null(gps->gpsd);
    retval = uquad_mat_zeros(gps->vel);
    cleanup_if(retval);
    gps->vel_ep = uquad_mat_alloc(3,1);
    cleanup_if_null(gps->gpsd);
    uquad_mat_zeros(gps->vel_ep);
    cleanup_if(retval);

    m1x3 = uquad_mat_alloc(1,3);
    cleanup_if_null(m1x3);    

    // Initialize data structure and open connection
    // Use default host/port (NULL/0)
    retval = gps_open(NULL, 0, gps->gpsd);
    if(retval < 0)
    {
	err_log("GPS init failed, could not open connection to GPS, is daemon running?");
	cleanup_if(ERROR_GPS);
    }

    // Start TX from GPS
    retval = gps_stream(gps->gpsd, GPS_COMM_STREAM_FLAGS_ENA, NULL);
    if(retval < 0)
    {
	cleanup_log_if(ERROR_GPS,"GPS init failed, could not stream from GPS.");
    }
    return gps;

    cleanup:
    gps_comm_deinit(gps);
    return NULL;
}   

void  gps_comm_deinit(gps_t * gps){
    int retval;
    if(gps == NULL)
    {
	err_log("WARN: Nothing to free");
	return;
    }
    uquad_mat_free(gps->pos);
    uquad_mat_free(gps->pos_ep);
    uquad_mat_free(gps->vel);
    uquad_mat_free(gps->vel_ep);
    uquad_mat_free(m1x3);

    retval = gps_stream(gps->gpsd, GPS_COMM_STREAM_FLAGS_DIS, NULL);
    if(retval < 0)
	err_log("WARN: ignoring error while terminating GPS stream...");
    retval = gps_close (gps->gpsd);
    if(retval < 0)
	err_log("WARN: ignoring error while closing GPS...");
    free(gps->gpsd);
    free(gps);
}

int gps_comm_deg2utm(utm_t *utm, double la, double lo)
{
    if(utm == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument!");
    }
    double lat,lon;
    double Huso,S,deltaS;
    double a,epsilon,nu,v,ta,a1,a2,j2,j4,j6,alfa,beta,gama,Bm;
    char let;
    lat = deg2rad(la);
    lon = deg2rad(lo);

    Huso = floor( ( lo / 6 ) + 31);
    S = ( ( Huso * 6 ) - 183 );
    deltaS = lon - deg2rad(S);

    if (la<-72)      let='C';
    else if (la<-64) let='D';
    else if (la<-56) let='E';
    else if (la<-48) let='F';
    else if (la<-40) let='G';
    else if (la<-32) let='H';
    else if (la<-24) let='J';
    else if (la<-16) let='K';
    else if (la<-8)  let='L';
    else if (la<0)   let='M';
    else if (la<8)   let='N';
    else if (la<16)  let='P';
    else if (la<24)  let='Q';
    else if (la<32)  let='R';
    else if (la<40)  let='S';
    else if (la<48)  let='T';
    else if (la<56)  let='U';
    else if (la<64)  let='V';
    else if (la<72)  let='W';
    else             let='X';

    a = cos(lat) * sin(deltaS);
    epsilon = 0.5 * log( ( 1 + a) / ( 1 - a ) );
    nu = atan( tan(lat) / cos(deltaS) ) - lat;
    v = ( deg2utm_c / sqrt( ( 1 + ( deg2utm_ee * uquad_square(cos(lat)) ) ) )) * 0.9996;
    ta = ( deg2utm_ee / 2 ) * uquad_square(epsilon) * uquad_square(cos(lat));
    a1 = sin( 2 * lat );
    a2 = a1 * uquad_square(cos(lat));
    j2 = lat + ( a1 / 2 );
    j4 = ( ( 3 * j2 ) + a2 ) / 4;
    j6 = ( ( 5 * j4 ) + ( a2 * uquad_square(cos(lat))) ) / 3;
    alfa = ( 3 / 4 ) * deg2utm_ee;
    beta = ( 5 / 3 ) * uquad_square(alfa);
    gama = ( 35 / 27 ) * (alfa * alfa * alfa);
    Bm = 0.9996 * deg2utm_c * ( lat - alfa * j2 + beta * j4 - gama * j6 );
    utm->easting  = epsilon * v * ( 1 + ( ta / 3 ) ) + 500000;
    utm->northing = nu * v * ( 1 + ta ) + Bm;

    if (utm->northing < 0)
	utm->northing += 9999999;

    utm->let  = let;
    utm->zone = Huso;

    return ERROR_OK;
}

int gps_comm_get_fix_mode(gps_t *gps){
    return gps->gpsd->fix.mode;
}

int gps_comm_get_fd(gps_t *gps){
    return gps->gpsd->gps_fd;
}

int gps_comm_read(gps_t *gps){
    int retval;
    struct gps_fix_t gps_fix;
    retval = gps_read(gps->gpsd);
    if(retval == -1){
	err_check(ERROR_IO,"New data from expected, but none found...\nWas select(gps_fd) called before gps_comm_read()?");
    }
    retval = gps_comm_get_fix_mode(gps);
    if(retval < MODE_3D)
    {
	err_log_num("Fix:",retval);
	err_check(ERROR_GPS,"3D fix NOT available!");
    }
    gettimeofday(&gps->timestamp,NULL);
    gps->fix = gps_comm_get_fix_mode(gps);
    gps_fix = gps->gpsd->fix;
    retval = gps_comm_deg2utm(&gps->utm, gps_fix.latitude, gps_fix.longitude);
    err_propagate(retval);
    gps->pos->m_full[0] = gps->utm.easting;
    gps->pos->m_full[1] = gps->utm.northing;
    gps->pos->m_full[2] = gps_fix.altitude;

    if(isnan(gps_fix.epx) ||
       isnan(gps_fix.epy) ||
       isnan(gps_fix.epv))
    {
	gps->pos_ep_ok = false;
    }
    else
    {
	gps->unread_data = true;
	gps->pos_ep_ok = true;
	gps->pos_ep->m_full[0] = gps_fix.epx;
	gps->pos_ep->m_full[1] = gps_fix.epy;
	gps->pos_ep->m_full[2] = gps_fix.epv;
    }

    if(isnan(gps_fix.speed) ||
       isnan(gps_fix.climb) ||
       isnan(gps_fix.track))
    {
	gps->vel_ok = false;
    }
    else
    {
	gps->vel_ok = true;
	gps->speed = gps_fix.speed;
	gps->climb = gps_fix.climb;
	gps->track = deg2rad(gps_fix.track);
	if(isnan(gps_fix.eps) ||
	   isnan(gps_fix.epc) ||
	   isnan(gps_fix.epd))
	{
	    gps->vel_ep_ok = false;
	}
	else
	{
	    gps->vel_ep_ok = true;
	    gps->speed_ep = gps_fix.eps;
	    gps->climb_ep = gps_fix.epc;
	    gps->track_ep = deg2rad(gps_fix.epd);
	}
    }
    return ERROR_OK;
}

int gps_comm_get_data(gps_t *gps, uquad_mat_t *pos, uquad_mat_t *vel, imu_data_t *imu_data)
{
    int retval = ERROR_OK;
    if(pos == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument!");
    }
    uquad_mat_copy(pos, gps->pos);
    err_propagate(retval);
    if(vel != NULL)
    {
	if(!gps->vel_ok)
	{
	    err_check(ERROR_GPS_NO_VEL,"Speed update not available!");
	}
	if(imu_data == NULL)
	{
	    err_check(ERROR_GPS_SYS_REF,"Cannot convert from inertial to quad without IMU info!");
	}
	err_check(ERROR_FAIL,"Not impleneted!");//TODO implement!
	uquad_mat_copy(vel, gps->vel);
	err_propagate(retval);
    }
    return ERROR_OK;
}

int gps_comm_get_data_unread(gps_t *gps, uquad_mat_t *pos, uquad_mat_t *vel, imu_data_t *imu_data)
{
    int retval = ERROR_OK;
    if(!gps->unread_data)
    {
	err_check(ERROR_GPS_NO_UPDATES,"NO new data!");
    }
    retval = gps_comm_get_data(gps, pos, vel, imu_data);
    err_propagate(retval);
    gps->unread_data = false;
    return ERROR_OK;
}

void gps_comm_dump(gps_t *gps, FILE *stream)
{
    int retval = ERROR_OK;
    int i;
    retval = uquad_mat_transpose(m1x3, gps->pos);
    log_n_jump(retval,gps_comm_dump_end,"Failed to dump gps data!");    
    log_tv_only(stream,gps->timestamp);
    log_int_only(stream, gps->fix);
    for(i = 0; i < 3; ++i)
	log_double_only(stream,gps->vel->m_full[i]);
    uquad_mat_dump(m1x3, stream);
    gps_comm_dump_end:;
}
