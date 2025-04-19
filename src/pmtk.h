/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h>

// CR \r
// LF \n
// <CR><LF> \r\n
// $[....]<CR><LF>

/////////////////////////////////////////////////////////////////////////////
// PMTK001 ACK
// PMTK01,CMD,FLAG*CHECKSUM
// FLAG: 0-invalid, 1-unsupported, 2-valid,failed, 3-valid,success
// $PMTK001,604,3*32<CR><LF>
//
// $PMTK001,314,3*36 -> (314) set nema output, PMTK::RMCGGA
// $PMTK001,220,3*30 -> (220) set pos fix, PMTK::UPDATE_1HZ
/////////////////////////////////////////////////////////////////////////////

#if defined __cplusplus
extern "C" {
#endif

// PMTK commands
// https://www.sparkfun.com/datasheets/GPS/Modules/PMTK_Protocol.pdf
constexpr uint8_t PMTK_UPDATE_1HZ[] = "$PMTK220,1000*1F\r\n"; //  1 Hz
constexpr uint8_t PMTK_UPDATE_2HZ[] = "$PMTK220,500*2B\r\n";  //  2 Hz
constexpr uint8_t PMTK_UPDATE_5HZ[] =
    "$PMTK220,200*2C\r\n"; //  5 Hz ... invalid?
// constexpr uint8_t UPDATE_10HZ[] = "$PMTK220,100*2F\r\n"; // 10 Hz ...
// invalid? must > 200, only 100

// power modes
constexpr uint8_t PMTK_FULL_POWER[] =
    "$PMTK225,0*2B\r\n"; // full pwr/continuous

// Position fix update rates
constexpr uint8_t PMTK_FIX_CTL_1HZ[] =
    "$PMTK300,1000,0,0,0,0*1C\r\n"; // 1 Hz output
constexpr uint8_t PMTK_FIX_CTL_5HZ[] =
    "$PMTK300,200,0,0,0,0*2F\r\n"; // 5 Hz output

constexpr uint8_t PMTK_BAUD_115200[] = "$PMTK251,115200*1F\r\n"; // 115200 bps
constexpr uint8_t PMTK_BAUD_57600[]  = "$PMTK251,57600*2C\r\n";  //  57600 bps
constexpr uint8_t PMTK_BAUD_9600[]   = "$PMTK251,9600*17\r\n";   //   9600 bps

constexpr uint8_t PMTK_ANTENNA[] =
    "$PGCMD,33,1*6C\r\n"; // request for updates on antenna status
constexpr uint8_t PMTK_NOANTENNA[] =
    "$PGCMD,33,0*6D\r\n"; // don't show antenna status messages

constexpr uint8_t PMTK_ENABLE_SBAS[] =
    "$PMTK313,1*2E\r\n"; // Enable search for SBAS satellite (only works with
                         // 1Hz < output rate)
constexpr uint8_t PMTK_ENABLE_WAAS[] =
    "$PMTK301,2*2E\r\n"; // WAAS for DGPS correction data

constexpr uint8_t PMTK_GLL[] =
    "$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // only the GLL
                                                             // sentence
constexpr uint8_t PMTK_RMC[] =
    "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // only the RMC
                                                             // sentence
constexpr uint8_t PMTK_VTG[] =
    "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // only the VTG
constexpr uint8_t PMTK_GGA[] =
    "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // just the GGA
constexpr uint8_t PMTK_GSA[] =
    "$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // just the GSA
constexpr uint8_t PMTK_GSV[] =
    "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // just the GSV
constexpr uint8_t PMTK_RMCGGA[] =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; // RMC and GGA
constexpr uint8_t PMTK_RMCGGAGSA[] =
    "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // RMC, GGA and GSA

#if defined __cplusplus
}
#endif
