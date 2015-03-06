/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* this implements a GPS hardware library for the Android emulator.
 * the following code should be built as a shared library that will be
 * placed into /system/lib/hw/gps.goldfish.so
 *
 * it will be loaded by the code in hardware/libhardware/hardware.c
 * which is itself called from android_location_GpsLocationProvider.cpp
 */


#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>

#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <termios.h> 
#include <errno.h>

#define IOCTL_MAGIC		'S'
#define SERIAL_SEL_GPS		_IO(IOCTL_MAGIC, 1)
#define POWER_GPS		_IOW(IOCTL_MAGIC, 4, unsigned long)

#define  LOG_TAG  "GPSGTS4E60"
#define  GPS_DEBUG  1

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <hardware/gps.h>
#include <hardware/hardware.h>

/* the name of the controlled socket */
#define  CHANNEL_NAME  "gps"

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  16

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;


/* this is the state of our connection to the gps daemon */
typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            callbacks;
    pthread_t               thread;
    int                     control[2];
} GpsState;

static GpsState  _gps_state[1];

GpsStatus g_status;

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;

        if (q >= p) {
            if (count < MAX_NMEA_TOKENS) {
                t->tokens[count].p   = p;
                t->tokens[count].end = q;
                count += 1;
            }
        }
        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}

static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;
    char  temp[16];

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

#define  NMEA_MAX_SIZE  83

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;
	GpsSvStatus  sv_status;
   
    int     sv_status_changed; 
	GpsCallbacks callback;
    char    in[ NMEA_MAX_SIZE+1 ];
} NmeaReader;

static void
nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));

    r->utc_diff = time_local - time_utc;
}

static void
nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
    r->callback.sv_status_cb = NULL;
    r->callback.nmea_cb = NULL;
    r->callback.location_cb = NULL;
    r->callback.status_cb = NULL;
    r->fix.size = sizeof(r->fix);

    nmea_reader_update_utc_diff( r );
}

static void
nmea_reader_set_callback( NmeaReader*  r, gps_location_callback  cb )
{
	r->callback.location_cb  = cb;
	if (cb != NULL && r->fix.flags != 0) {
		LOGD_IF(GPS_DEBUG, "%s: sending latest fix to new callback", __FUNCTION__);
		r->callback.location_cb( &r->fix );
    }
}

static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     fix_time;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        // no date yet, get current one
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        r->utc_year = tm.tm_year + 1900;
        r->utc_mon  = tm.tm_mon + 1;
        r->utc_day  = tm.tm_mday;
    }

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour  = hour;
    tm.tm_min   = minute;
    tm.tm_sec   = (int) seconds;
    tm.tm_year  = r->utc_year - 1900;
    tm.tm_mon   = r->utc_mon - 1;
    tm.tm_mday  = r->utc_day;
    tm.tm_isdst = -1;

    fix_time = mktime( &tm ) + r->utc_diff;
    r->fix.timestamp = (long long)fix_time * 1000;
    return 0;
}
 
static int
nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
{
 
    if ( (tok_d.p + 2 > tok_d.end) ||
         (tok_m.p + 2 > tok_m.end) ||
         (tok_y.p + 4 > tok_y.end) )
        return -1;
 
    r->utc_day = str2int(tok_d.p,   tok_d.p+2);
    r->utc_mon = str2int(tok_m.p, tok_m.p+2);
    r->utc_year = str2int(tok_y.p, tok_y.end+4);
 
    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end) {
        LOGE("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;

    if ((day|mon|year) < 0) {
        LOGE("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }

    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, time );
}


static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        LOGE("latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        LOGE("longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end);
    return 0;
}

static int nmea_reader_update_accuracy(NmeaReader * r,  Token accuracy)
{
	double acc;
	Token  tok = accuracy;

	if(tok.p >= tok.end)
		return -1;

	r->fix.accuracy = str2float(tok.p, tok.end);

	if(r->fix.accuracy == 99.99){
	return 0;
	}

	r->fix.flags |= GPS_LOCATION_HAS_ACCURACY;
	return 0;
}

static void
nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;

    LOGD_IF(GPS_DEBUG, "Received: '%.*s'", r->pos, r->in);
    if (r->pos < 9) {
        LOGE("Too short. discarded.");
        return;
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
    {
        int  n;
        LOGD_IF(GPS_DEBUG, "Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            LOGD_IF(GPS_DEBUG, "%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }

    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end) {
        LOGE("sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
        return;
    }

    // ignore first two characters.
    tok.p += 2;
    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
        Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
        Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

        nmea_reader_update_time(r, tok_time);
        nmea_reader_update_latlong(r, tok_latitude,
                                      tok_latitudeHemi.p[0],
                                      tok_longitude,
                                      tok_longitudeHemi.p[0]);
        nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);

    } else if ( !memcmp(tok.p, "GSA", 3) ) {
        // do something ?
        {  
	LOGD_IF(GPS_DEBUG, "may%s,%d,%s,gsa\n",__FILE__,__LINE__,__FUNCTION__);  
        Token tok_fixStatus = nmea_tokenizer_get(tzer, 2);  
        int i;  
  
        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {  
            Token tok_accuracy = nmea_tokenizer_get(tzer, 15);//position dilution of precision dop   
            nmea_reader_update_accuracy(r, tok_accuracy);  
            r->sv_status.used_in_fix_mask = 0ul;  

            for (i = 3; i <= 14; ++i){  
                Token tok_prn = nmea_tokenizer_get(tzer, i);  
                int prn = str2int(tok_prn.p, tok_prn.end);  
      		  LOGD_IF(GPS_DEBUG, "gsa,prn=%d,",prn);  
                if (prn > 0){  
                    r->sv_status.used_in_fix_mask |= (1ul << ( prn-1));  
                    r->sv_status_changed = 1;  
               }  
           }
       LOGD_IF(GPS_DEBUG, "%s: fix mask is %x", __FUNCTION__, r->sv_status.used_in_fix_mask);  
       }  
       LOGD_IF(GPS_DEBUG, " [log hit][%s:%d] fix.flags=0x%x ", __FUNCTION__, __LINE__, r->fix.flags);  
    }  

    } //////////////////////////////////////////////////////////////////////////////////////////////////
    else if ( !memcmp(tok.p, "GSV", 3) ) {  
	LOGD_IF(GPS_DEBUG, "may%s,%d,%s,gsV\n",__FILE__,__LINE__,__FUNCTION__);  
    Token tok_noSatellites = nmea_tokenizer_get(tzer, 3);  
    int noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);  
       LOGD_IF(GPS_DEBUG, "%d,inview=%d,\n",__LINE__,noSatellites);    
    if (noSatellites > 0) {  
     Token tok_noSentences = nmea_tokenizer_get(tzer, 1);  
     Token tok_sentence     = nmea_tokenizer_get(tzer, 2);  
      
     int sentence = str2int(tok_sentence.p, tok_sentence.end);  
     int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);  
	LOGD_IF(GPS_DEBUG, "%d,gsv_index=%d,gsv_total=%d\n",__LINE__,sentence,totalSentences);    
     int curr;  
     int i;  
               
     if (sentence == 1) {  
	LOGD_IF(GPS_DEBUG, "msg_index=%d\n",sentence);  
   //  r->sv_status_changed = 0;   
     r->sv_status.num_svs = 0;  
    r->sv_status.ephemeris_mask=0ul;  
    r->sv_status.almanac_mask=0ul;  
     }  
      
        curr = r->sv_status.num_svs;  
     
        i = 0;  
      
        while (i < 4 && r->sv_status.num_svs < noSatellites){  
         Token    tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);  
         Token    tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);  
         Token    tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);  
         Token    tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);  
      
         r->sv_status.sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);  
         r->sv_status.sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);  
         r->sv_status.sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);  
         r->sv_status.sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);  
         r->sv_status.ephemeris_mask|=(1ul << (r->sv_status.sv_list[curr].prn-1));  
    r->sv_status.almanac_mask|=(1ul << (r->sv_status.sv_list[curr].prn-1));           
    r->sv_status.num_svs += 1;  
	LOGD_IF(GPS_DEBUG, "**********curr=%d\n",curr);  
   
	LOGD_IF(GPS_DEBUG, "%d,prn=%d:snr=%f\n",__LINE__,r->sv_status.sv_list[curr].prn,r->sv_status.sv_list[curr].snr);  
         curr += 1;  
      
         i += 1;  
     }  
      
     if (sentence == totalSentences) {  
	LOGD_IF(GPS_DEBUG, "msg=%d,msgindex=%d",totalSentences,sentence);  
	r->callback.sv_status_cb=_gps_state->callbacks.sv_status_cb;  
  
     if (r->sv_status_changed !=0) {  
           if (r->callback.sv_status_cb) {  
          
        LOGD_IF(GPS_DEBUG, "%d,SV_STATSU,change=%d\n",__LINE__,r->sv_status_changed);  
        int nums=r->sv_status.num_svs;  
        LOGD_IF(GPS_DEBUG, "num_svs=%d,emask=%x,amask=%x,inusemask=%x\n",r->sv_status.num_svs,r->sv_status.ephemeris_mask,r->sv_status.almanac_mask,r->sv_status.used_in_fix_mask);  
        LOGD_IF(GPS_DEBUG, "************88\n");        
        while(nums)  
        {  
        nums--;  
        LOGD_IF(GPS_DEBUG, "prn=%d:snr=%f\n",r->sv_status.sv_list[nums].prn,r->sv_status.sv_list[nums].snr);  
          
        }LOGD_IF(GPS_DEBUG, "************88\n");  
                 r->callback.sv_status_cb( &(r->sv_status) );  
               r->sv_status_changed = 0;  
             }else {  
                LOGE("no callback, keeping status data until needed !");  
           }  
  
        }  
     }  
      
     LOGD_IF(GPS_DEBUG, "%s: GSV message with total satellites %d", __FUNCTION__, noSatellites);   
      
    }           
      
    }  

	else if ( !memcmp(tok.p, "GLL", 3) ) {  
		    Token tok_fixstaus = nmea_tokenizer_get(tzer,6);  
		    if (tok_fixstaus.p[0] == 'A') {  
		     Token tok_latitude = nmea_tokenizer_get(tzer,1);  
		     Token tok_latitudeHemi = nmea_tokenizer_get(tzer,2);  
		     Token tok_longitude = nmea_tokenizer_get(tzer,3);  
		     Token tok_longitudeHemi = nmea_tokenizer_get(tzer,4);  
		     Token tok_time = nmea_tokenizer_get(tzer,5);  
		     nmea_reader_update_time(r, tok_time);  
		     nmea_reader_update_latlong(r, tok_latitude, tok_latitudeHemi.p[0], tok_longitude, tok_longitudeHemi.p[0]);  
		    }  
    } else if ( !memcmp(tok.p, "VTG", 3) ) {
 
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,9);
 
        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N')
        {
            Token  tok_bearing       = nmea_tokenizer_get(tzer,1);
            Token  tok_speed         = nmea_tokenizer_get(tzer,5);
 
            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }
 
    } else if ( !memcmp(tok.p, "ZDA", 3) ) {
 
        Token  tok_time;
        Token  tok_year  = nmea_tokenizer_get(tzer,4);
 
        if (tok_year.p[0] != '\0') {
 
          Token  tok_day   = nmea_tokenizer_get(tzer,2);
          Token  tok_mon   = nmea_tokenizer_get(tzer,3);
 
          nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );
 
        }
 
        tok_time  = nmea_tokenizer_get(tzer,1);
 
        if (tok_time.p[0] != '\0') {
 
          nmea_reader_update_time(r, tok_time);
 
        }
    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        Token tok_time = nmea_tokenizer_get(tzer,1);  
        Token tok_fixStatus = nmea_tokenizer_get(tzer,2);  
        Token tok_latitude = nmea_tokenizer_get(tzer,3);  
        Token tok_latitudeHemi = nmea_tokenizer_get(tzer,4);  
        Token tok_longitude = nmea_tokenizer_get(tzer,5);  
        Token tok_longitudeHemi = nmea_tokenizer_get(tzer,6);  
        Token tok_speed = nmea_tokenizer_get(tzer,7);  
        Token tok_bearing = nmea_tokenizer_get(tzer,8);  
        Token tok_date = nmea_tokenizer_get(tzer,9);  
  
        LOGD_IF(GPS_DEBUG, "in RMC, fixStatus=%c", tok_fixStatus.p[0]);  
       if (tok_fixStatus.p[0] == 'A')  
        {  
            nmea_reader_update_date( r, tok_date, tok_time );  
  
            nmea_reader_update_latlong( r, tok_latitude,  
                                           tok_latitudeHemi.p[0],  
                                           tok_longitude,  
                                           tok_longitudeHemi.p[0] );  
  
            nmea_reader_update_bearing( r, tok_bearing );  
            nmea_reader_update_speed ( r, tok_speed );  

		r->callback.location_cb=_gps_state->callbacks.location_cb;  
		r->callback.nmea_cb=_gps_state->callbacks.nmea_cb;  
		r->callback.status_cb=_gps_state->callbacks.status_cb;  
		if (r->callback.status_cb) {  
		LOGD_IF(GPS_DEBUG, "report,status,flags=%d\n",r->fix.flags);  
		            r->callback.status_cb(&g_status);  
		        }  
		     if (r->callback.location_cb) {  
		LOGD_IF(GPS_DEBUG, "location_cb report:r->fix.flags=%d,r->latitude=%f,r->longitude=%f,r->altitude=%f,r->speed=%f,r->bearing=%f,r->accuracy=%f\n",r->fix.flags,r->fix.latitude,r->fix.longitude,r->fix.altitude,r->fix.speed,r->fix.bearing,r->fix.accuracy);  
		            r->callback.location_cb( &r->fix );  
		LOGD_IF(GPS_DEBUG, "line:%d\n",__LINE__);  
		        r->fix.flags = 0;  
		          
		        }  
		if (r->callback.nmea_cb) {  
		LOGD_IF(GPS_DEBUG, "report,timestamp=%llx,%llu\n",r->fix.timestamp,r->fix.timestamp);  
		            r->callback.nmea_cb( r->fix.timestamp,r->in,r->pos );  
          
             
        }  
  
        }  
    } else {
        tok.p -= 2;
        LOGE("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }
    if (r->fix.flags != 0) {
        char   temp[256];
        char*  p   = temp;
        char*  end = p + sizeof(temp);
        struct tm   utc;

        p += snprintf( p, end-p, "sending fix" );
        if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
            p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
            p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
            p += snprintf(p, end-p, " speed=%g", r->fix.speed);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
            p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
            p += snprintf(p,end-p, " accuracy=%g", r->fix.accuracy);
        }
        gmtime_r( (time_t*) &r->fix.timestamp, &utc );
        p += snprintf(p, end-p, " time=%s", asctime( &utc ) );
        LOGD_IF(GPS_DEBUG, "%s\n", temp);
    }
}


static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        nmea_reader_parse( r );
        r->pos = 0;
    }
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/
static void gps_state_init( GpsState*  state );
static void gps_state_done( GpsState*  s );

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};

static int
epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static void
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    NmeaReader  reader[1];
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];

    nmea_reader_init( reader );

    // register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    epoll_register( epoll_fd, gps_fd );

    LOGD_IF(GPS_DEBUG, "gps thread running");

    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_fd, events, 2, -1 );
        if (nevents < 0) {
            if (errno != EINTR)
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
        LOGD_IF(GPS_DEBUG, "gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    LOGD_IF(GPS_DEBUG, "gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        LOGD_IF(GPS_DEBUG, "gps thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START) {
                        if (!started) {
                            LOGD_IF(GPS_DEBUG, "gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
				g_status.status=GPS_STATUS_SESSION_BEGIN;
				state->callbacks.status_cb(&g_status); 
                            nmea_reader_set_callback( reader, state->callbacks.location_cb );
                        }
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            LOGD_IF(GPS_DEBUG, "gps thread stopping");
                            started = 0;
				g_status.status=GPS_STATUS_SESSION_END;
				state->callbacks.status_cb(&g_status); 
                            nmea_reader_set_callback( reader, NULL );
                        }
                    }
                }
                else if (fd == gps_fd)
                {
                    char  buff[32];
                    LOGD_IF(GPS_DEBUG, "gps fd event");
                    for (;;) {
                        int  nn, ret;

                        ret = read( fd, buff, sizeof(buff) );
                        if (ret < 0) {
                            if (errno == EINTR)
                                continue;
                            if (errno != EWOULDBLOCK)
                                LOGE("error while reading from gps daemon socket: %s:", strerror(errno));
                            break;
                        }
                        LOGD_IF(GPS_DEBUG, "received %d bytes: %.*s", ret, ret, buff);
#if 1
                        for (nn = 0; nn < ret; nn++)
                            nmea_reader_addc( reader, buff[nn] );
#else
{
#define GPS_TestData "$GPRMC,053322.682,A,2502.6538,N,12121.4838,E,0.00,080905,,,A*6F\n"

    char testbuf[100];
    unsigned int nn;
    sprintf(testbuf,"%s",GPS_TestData);
    for (nn = 0; nn < strlen(testbuf); nn++)
{
   // D("%02x ",buff[nn]);
                    nmea_reader_addc( reader, testbuf[nn] );
}

}
#endif
                    }
                    LOGD_IF(GPS_DEBUG, "gps fd event end");
                }
                else
                {
                    LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
Exit:
    return;
}

static void
gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        LOGE("%s: could not send CMD_START command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}

static int
gps_start()
{
    GpsState*  s = _gps_state;

    if (!s->init)
        gps_state_init(s);

    if (s->fd < 0){
        LOGE("%s: s->fd < 0 !!\n", __FUNCTION__);
        return -1;
    }
	
    if (!s->init) {
        LOGE("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    LOGI("%s: called", __FUNCTION__);

    gps_state_start(s);
    return 0;
}

static void
gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        LOGE("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}

static int
gps_stop()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        LOGE("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }
    
    LOGI("%s: called", __FUNCTION__);
    gps_state_stop(s);
    gps_state_done(s);
    return 0;
}

static void
gps_port_deinit( GpsState*  s )
{
	int serial_sel_fd;

	close( s->fd ); s->fd = -1;

//	usleep(100);
	serial_sel_fd = open("/dev/serial_sel", O_RDWR);
	if(serial_sel_fd < 0){
		LOGE("open /dev/serial_sel fail\n");
		return;
	}

	ioctl(serial_sel_fd, POWER_GPS, 0);
	close(serial_sel_fd);
}

static void
gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    int   ret;
    char   cmd = CMD_QUIT;
    void*  dummy;
    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        LOGE("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
	
    pthread_join(s->thread, &dummy);

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // close connection to the GPS daemon
    gps_port_deinit( s );
    s->init = 0;
}

static void
gps_port_init( GpsState*  state )
{
    int serial_sel_fd;

	state->fd         = -1;
	struct termios gps_termios;

	state->fd = open("/dev/ttyXM0",O_RDWR);
// 	state->fd = open("/dev/s3c2410_serial0",O_RDWR);
	if(state->fd < 0){
		LOGE("open /dev/ttyXM0 failed\n");
		return;
	}

	LOGI("port setup start...\n");

	tcgetattr(state->fd,&gps_termios);
	cfmakeraw(&gps_termios);
	cfsetispeed(&gps_termios,B9600);
	cfsetospeed(&gps_termios,B9600);
	tcsetattr(state->fd,TCSANOW,&gps_termios);
	tcflush(state->fd, TCIOFLUSH);

	serial_sel_fd = open("/dev/serial_sel", O_RDWR);
	if(serial_sel_fd < 0){
		LOGE("open /dev/serial_sel fail\n");
		return;
	}

	ioctl(serial_sel_fd, POWER_GPS, 1);
	ioctl(serial_sel_fd, SERIAL_SEL_GPS);
	close(serial_sel_fd);
//	usleep(100);

	LOGI("port setup finished...\n");

	LOGD_IF(GPS_DEBUG, "gps will read from '%s' channel", CHANNEL_NAME );
}

static void
gps_state_init( GpsState*  state )
{
	state->init       = 1;
	state->control[0] = -1;
	state->control[1] = -1;

	gps_port_init(state);

	if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
		LOGE("could not create thread control socket pair: %s", strerror(errno));
		goto Fail;
	}

//	if ( pthread_create( &state->thread, NULL, gps_state_thread, state ) != 0 ) {
//		LOGE("could not create gps thread: %s", strerror(errno));
//		goto Fail;
//	}
	state->thread=state->callbacks.create_thread_cb("gps_state_thread",gps_state_thread,state);

	LOGI("gps state initialized");
	return;

Fail:
	gps_state_done( state );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static int
gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;
    s->callbacks = *callbacks;

    g_status.status=GPS_STATUS_ENGINE_ON;
    s->callbacks.status_cb(&g_status);
return 0;
    if (!s->init)
        gps_state_init(s);

    if (s->fd < 0){
        LOGE("%s: s->fd < 0 !!\n", __FUNCTION__);
        return -1;
    }

    return 0;
}

static void
gps_cleanup(void)
{
    GpsState*  s = _gps_state;
return;
    if (s->init)
        gps_state_done(s);
}

static int
gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

static int
gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}

static void
gps_delete_aiding_data(GpsAidingData flags)
{
}

static int gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
            uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    // FIXME - support fix_frequency
    return 0;
}

static const void*
gps_get_extension(const char* name)
{
    // no extensions supported
    return NULL;
}

static const GpsInterface  gpsInterface = {
    sizeof(GpsInterface),
    gps_init,
    gps_start,
    gps_stop,
    gps_cleanup,
    gps_inject_time,
    gps_inject_location,
    gps_delete_aiding_data,
    gps_set_position_mode,
    gps_get_extension,
};

const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev)
{
    return &gpsInterface;
}

static int open_gps(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device)
{
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    dev->get_gps_interface = gps__get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}


static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

const struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "Goldfish GPS Module",
    .author = "The Android Open Source Project",
    .methods = &gps_module_methods,
};
