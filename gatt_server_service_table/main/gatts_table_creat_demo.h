/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_ilong,
    IDX_CHAR_VAL_ilong,
    IDX_CHAR_CFG_ilong,

    IDX_CHAR_ilat,
    IDX_CHAR_VAL_ilat,

    IDX_CHAR_flong,
    IDX_CHAR_VAL_flong,

    IDX_CHAR_flat,
    IDX_CHAR_VAL_flat,

    IDX_CHAR_xAccel,
    IDX_CHAR_VAL_xAccel,

    IDX_CHAR_yAccel,
    IDX_CHAR_VAL_yAccel,

    IDX_CHAR_zAccel,
    IDX_CHAR_VAL_zAccel,

    IDX_CHAR_xGyro,
    IDX_CHAR_VAL_xGyro,

    IDX_CHAR_yGyro,
    IDX_CHAR_VAL_yGyro,

    IDX_CHAR_zGyro,
    IDX_CHAR_VAL_zGyro,

/*    IDX_CHAR_avgRPS,
    IDX_CHAR_VAL_avgRPS,
    
    IDX_CHAR_peakRPS,
    IDX_CHAR_VAL_peakRPS, */

    IDX_CHAR_ialt,
    IDX_CHAR_VAL_ialt,

    IDX_CHAR_falt,
    IDX_CHAR_VAL_falt,

    HRS_IDX_NB,
};
