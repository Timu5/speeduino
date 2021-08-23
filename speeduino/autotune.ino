/*
Copyright (C) 2021 Mateusz Muszyński
A full copy of the license may be found in the projects root directory
*/

#include "globals.h"
#include "auxiliaries.h"
#include "maths.h"
#include "decoders.h"
#include "speeduino.h"
#include "timers.h"
#include "sensors.h"

#include "autotune.h"


void autotune()
{
    if (currentStatus.tpsDOT < 2)
    {
        uint16_t ratio = ((currentStatus.O2 * 128) / currentStatus.afrTarget);

        update3DTableValue(&fuelTable, currentStatus.fuelLoad, currentStatus.RPM, ratio);
    }
}