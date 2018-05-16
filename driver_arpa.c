/*
 * This file is Copyright (c) 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

/* for vsnprintf() FreeBSD wants __ISO_C_VISIBLE >= 1999 */
#define __ISO_C_VISIBLE 1999

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#include "bits.h"
#include "gpsd.h"
#include "strfuncs.h"

#ifdef NMEA0183_ENABLE
#ifdef ARPA_ENABLE

/**************************************************************************
 *
 * Compare GPS timestamps for equality.  Depends on the fact that the
 * timestamp granularity of GPS is 1/100th of a second.  Use this to avoid
 * naive float comparisons.
 *
 **************************************************************************/

#define GPS_TIME_EQUAL(a, b) (fabs((a) - (b)) < 0.01)

/**************************************************************************
 * NMEA sentence handling begins here
 **************************************************************************/
static gps_mask_t processTTM( int count UNUSED,
                              char *field[] UNUSED,
			      struct gps_device_t *session UNUSED)
/* Geographic position - Latitude, Longitude */
{
    /* TTM - Tracked Target Message
     * ============================
     *                                          11     13
     *         1  2   3   4 5   6   7 8   9   10|    12| 14
     *         |  |   |   | |   |   | |   |   | |    | | |
     * $--TTM,xx,x.x,x.x,a,x.x,x.x,a,x.x,x.x,a,c--c,a,a*hh<CR><LF>
     *
     * Field Number:
     *    1 - Target Number (0-99)
     *    2 - Target Distance
     *    3 - Bearing from own ship
     *    4 - Bearing Units
     *    5 - Target Speed
     *    6 - Target Course
     *    7 - Course Units
     *    8 - Distance of closest-point-of-approach
     *    9 - Time until closest-point-of-approach
     *   10 - "-" means increasing
     *   11 - Target name
     *   12 - Target Status
     *   13 - Reference Target
     *   14 - Checksum
     *
     * E.G.:
     *   $GPTTM,1,2.48,235.1,T,1.8,268.9,T,,,N,,Q,,000000,A*2E
     *        1    2     3 4   5     6 7  10 11        13 14
     *
     */

    // char *status = field[7];
    gps_mask_t mask = 0;

    // if (field[5][0] != '\0') {
	// merge_hhmmss(field[5], session);
	// register_fractional_time(field[0], field[5], session);
	// if (session->nmea.date.tm_year == 0)
	//     gpsd_log(&session->context->errout, LOG_WARN,
	// 	     "can't use GLL time until after ZDA or RMC has supplied a year.\n");
	// else {
	//     mask = TIME_SET;
	// }
    // }
    // if (strcmp(field[6], "A") == 0 && (count < 8 || *status != 'N')) {
	// int newstatus;
    //
	// do_lat_lon(&field[1], &session->newdata);
	// mask |= LATLON_SET;

	// session->gpsdata.status = newstatus;
	// mask |= STATUS_SET;
    // }

    // gpsd_log(&session->context->errout, LOG_DATA,
	//      "GLL: hhmmss=%s lat=%.2f lon=%.2f mode=%d status=%d\n",
	//      field[5],
	//      session->newdata.latitude,
	//      session->newdata.longitude,
	//      session->newdata.mode,
	//      session->gpsdata.status);
    return mask;
}

/*
 * A prototype driver.
 *
 *
 * Once that is done, you will likely have to define a large number of
 * flags and masks. From there, you will be able to start extracting
 * useful quantities. There are roughed-in decoders for the navigation
 * solution, satellite status and gps-utc offset. These are the 3 key
 * messages that gpsd needs.
 *
 *
 * You will also need to add hooks for your new driver to:
 * [y] SConstruct
 * [/] drivers.c
 * [?] gpsd.h-tail
 * [?] libgpsd_core.c
 *
 *
 * This file is Copyright (c) 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
static gps_mask_t processTTM( int count, char *field[], struct gps_device_t *session);
static	gps_mask_t _arpa_parse_input(struct gps_device_t *);


/*
 * These methods may be called elsewhere in gpsd
 */

static	bool _arpa_probe_detect(struct gps_device_t *);
static	void _arpa_event_hook(struct gps_device_t *, event_t);
static	bool _arpa_set_speed(struct gps_device_t *, speed_t, char, int);


/**
 * Parse the data from the device
 */
// gps_mask_t _arpa_dispatch(struct gps_device_t *session, unsigned char *buf, size_t len)
// {
//     size_t i;
//     int type, used, visible, retmask = 0;
//
//     if (len == 0)
// 	return 0;
//
//     /*
//      * Set this if the driver reliably signals end of cycle.
//      * The core library zeroes it just before it calls each driver's
//      * packet analyzer.
//      */
//     session->cycle_end_reliable = true;
//     if (msgid == MY_START_OF_CYCLE)
// 	retmask |= CLEAR_IS;
//     else if (msgid == MY_END_OF_CYCLE)
// 	retmask |= REPORT_IS;
//
//     type = GET_MESSAGE_TYPE();
//
//     /* we may need to dump the raw packet */
//     gpsd_log(&session->context->errout, LOG_RAW,
// 	     "raw _arpa_ packet type 0x%02x\n", type);
//
//     switch (type)
//     {
// 	/* Deliver message to specific decoder based on message type */
//
//     default:
// 	gpsd_log(&session->context->errout, LOG_WARN,
// 		 "unknown packet id %d length %d\n", type, len);
// 	return 0;
//     }
// }

/**********************************************************
 *
 * Externally called routines below here
 *
 **********************************************************/

static bool _arpa_probe_detect(struct gps_device_t *session UNUSED)
{
   /*
    * This method is used to elicit a positively identifying
    * response from a candidate device. Some drivers may use
    * this to test for the presence of a certain kernel module.
    */
   int test, satisfied;

   /* Your testing code here */
   test=satisfied=0;

   if (test==satisfied)
      return true;
   return false;
}

#ifdef RECONFIGURE_ENABLE
static void _arpa_event_hook(struct gps_device_t *session, event_t event)
{
    if (session->context->readonly)
	return;

    if (event == event_wakeup) {
       /*
	* Code to make the device ready to communicate.  Only needed if the
	* device is in some kind of sleeping state, and only shipped to
	* RS232C, so that gpsd won't send strings to unidentified USB devices
	* that might not be GPSes at all.
	*/
    }
    if (event == event_identified) {
	/*
	 * Fires when the first full packet is recognized from a
	 * previously unidentified device.  The session.lexer counter
	 * is zeroed.  If your device has a default cycle time other
	 * than 1 second, set session->device->gpsdata.cycle here. If
	 * possible, get the software version and store it in
	 * session->subtype.
	 */
    }
    if (event == event_configure) {
	/*
	 * Change sentence mix and set reporting modes as needed.
	 * Called immediately after event_identified fires, then just
	 * after every packet received thereafter, but you probably
	 * only want to take actions on the first few packets after
	 * the session.lexer counter has been zeroed,
	 *
	 * Remember that session->lexer.counter is available when you
	 * write this hook; you can use this fact to interleave configuration
	 * sends with the first few packet reads, which is useful for
	 * devices with small receive buffers.
	 */
    } else if (event == event_driver_switch) {
	/*
	 * Fires when the driver on a device is changed *after* it
	 * has been identified.
	 */
    } else if (event == event_deactivate) {
	/*
	 * Fires when the device is deactivated.  Usr this to revert
	 * whatever was done at event_identify and event_configure
	 * time.
	 */
    } else if (event == event_reactivate) {
       /*
	* Fires when a device is reactivated after having been closed.
	* Use this hook for re-establishing device settings that
	* it doesn't hold through closes.
	*/
    }
}

/*
 * This is the entry point to the driver. When the packet sniffer recognizes
 * a packet for this driver, it calls this method which passes the packet to
 * the binary processor or the nmea processor, depending on the session type.
 */
static gps_mask_t _arpa_parse_input(struct gps_device_t *session UNUSED)
{
    if (session->lexer.type == NMEA_PACKET) {
	return _arpa_dispatch(session, session->lexer.outbuffer, session->lexer.outbuflen);
#ifdef NMEA0183_ENABLE
    } else if (session->lexer.type == NMEA_PACKET) {
	return nmea_parse((char *)session->lexer.outbuffer, session);
#endif /* NMEA0183_ENABLE */
    } else
	return 0;
}

static bool _arpa_set_speed(struct gps_device_t *session UNUSED,
			      speed_t speed UNUSED, char parity UNUSED, int stopbits UNUSED)
{
    /*
     * Set port operating mode, speed, parity, stopbits etc. here.
     * Note: parity is passed as 'N'/'E'/'O', but you should program
     * defensively and allow 0/1/2 as well.
     */
     return false;
}
#endif /* RECONFIGURE_ENABLE */

static void _arpa_wrapup(struct gps_device_t *session UNUSED)
{
}

#endif /* ARPA_ENABLE */
#endif /* NMEA0183_ENABLE */
