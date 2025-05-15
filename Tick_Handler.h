#ifndef _TICK_HANDLER_H
#define _TICK_HANDLER_H
#include "AP_struct.h"

/*Tick Handler Header fnt*/
void Tick_Handle(void);

_Tick_t TICK;

void Tick_Handle(void)
{
    TICK.flag.bit_F = false;

    if (++TICK.cnt_ms2 > 2)
    {
        TICK.flag.bit_flag.ms2 = true;
        TICK.cnt_ms2 = 0;
    }

    if (++TICK.cnt_ms5 > 5)
    {
        TICK.flag.bit_flag.ms5 = true;
        TICK.cnt_ms5 = 0;
    }
    if (++TICK.cnt_ms10 > 10)
    {
        TICK.flag.bit_flag.ms10 = true;
        TICK.cnt_ms10 = 0;
    }

    if (++TICK.cnt_ms25 > 25)
    {
        TICK.flag.bit_flag.ms25 = true;
        TICK.cnt_ms25 = 0;
    }

    if (++TICK.cnt_ms50 > 50)
    {
        TICK.flag.bit_flag.ms50 = true;
        TICK.cnt_ms50 = 0;
    }

    if (++TICK.cnt_ms100 > 100)
    {
        TICK.flag.bit_flag.ms100 = true;
        TICK.cnt_ms100 = 0;
    }

    if (++TICK.cnt_ms250 > 250)
    {
        TICK.flag.bit_flag.ms250 = true;
        TICK.cnt_ms250 = 0;
    }

    if (++TICK.cnt_ms500 > 500)
    {
        TICK.flag.bit_flag.ms500 = true;
        TICK.cnt_ms500 = 0;
    }

    if (++TICK.cnt_sec1 > 1000)
    {
        TICK.flag.bit_flag.sec1 = true;
        TICK.cnt_sec1 = 0;
    }
    if( ++TICK.cnt_sec2 > 2000)
    {
        TICK.flag.bit_flag.sec2 = true;
        TICK.cnt_sec2 = 0;
    }
}


#endif