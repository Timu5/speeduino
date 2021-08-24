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

// Based on get3DTableValue from table.ino
void update3DTableValue(struct table3D *fromTable, int Y_in, int X_in, uint16_t ratio)
  {
    int X = X_in;
    int Y = Y_in;

    int tableResult = 0;
    //Loop through the X axis bins for the min/max pair
    //Note: For the X axis specifically, rather than looping from tableAxisX[0] up to tableAxisX[max], we start at tableAxisX[Max] and go down.
    //      This is because the important tables (fuel and injection) will have the highest RPM at the top of the X axis, so starting there will mean the best case occurs when the RPM is highest (And hence the CPU is needed most)
    int xMinValue = fromTable->axisX[0];
    int xMaxValue = fromTable->axisX[fromTable->xSize-1];
    byte xMin = 0;
    byte xMax = 0;

    //If the requested X value is greater/small than the maximum/minimum bin, reset X to be that value
    if(X > xMaxValue) { X = xMaxValue; }
    if(X < xMinValue) { X = xMinValue; }

    //Commence the lookups on the X and Y axis

    //1st check is whether we're still in the same X bin as last time
    if ( (X <= fromTable->axisX[fromTable->lastXMax]) && (X > fromTable->axisX[fromTable->lastXMin]) )
    {
      xMaxValue = fromTable->axisX[fromTable->lastXMax];
      xMinValue = fromTable->axisX[fromTable->lastXMin];
      xMax = fromTable->lastXMax;
      xMin = fromTable->lastXMin;
    }
    //2nd check is whether we're in the next RPM bin (To the right)
    else if ( ((fromTable->lastXMax + 1) < fromTable->xSize ) && (X <= fromTable->axisX[fromTable->lastXMax +1 ]) && (X > fromTable->axisX[fromTable->lastXMin + 1]) ) //First make sure we're not already at the last X bin
    {
      xMax = fromTable->lastXMax + 1;
      fromTable->lastXMax = xMax;
      xMin = fromTable->lastXMin + 1;
      fromTable->lastXMin = xMin;
      xMaxValue = fromTable->axisX[fromTable->lastXMax];
      xMinValue = fromTable->axisX[fromTable->lastXMin];
    }
    //3rd check is to look at the previous bin (to the left)
    else if ( (fromTable->lastXMin > 0 ) && (X <= fromTable->axisX[fromTable->lastXMax - 1]) && (X > fromTable->axisX[fromTable->lastXMin - 1]) ) //First make sure we're not already at the first X bin
    {
      xMax = fromTable->lastXMax - 1;
      fromTable->lastXMax = xMax;
      xMin = fromTable->lastXMin - 1;
      fromTable->lastXMin = xMin;
      xMaxValue = fromTable->axisX[fromTable->lastXMax];
      xMinValue = fromTable->axisX[fromTable->lastXMin];
    }
    else
    //If it's not caught by one of the above scenarios, give up and just run the loop
    {
      for (int8_t x = fromTable->xSize-1; x >= 0; x--)
      {
        //Checks the case where the X value is exactly what was requested
        if ( (X == fromTable->axisX[x]) || (x == 0) )
        {
          xMaxValue = fromTable->axisX[x];
          xMinValue = fromTable->axisX[x];
          xMax = x;
          fromTable->lastXMax = xMax;
          xMin = x;
          fromTable->lastXMin = xMin;
          break;
        }
        //Normal case
        if ( (X <= fromTable->axisX[x]) && (X > fromTable->axisX[x-1]) )
        {
          xMaxValue = fromTable->axisX[x];
          xMinValue = fromTable->axisX[x-1];
          xMax = x;
          fromTable->lastXMax = xMax;
          xMin = x-1;
          fromTable->lastXMin = xMin;
          break;
        }
      }
    }

    //Loop through the Y axis bins for the min/max pair
    int yMaxValue = fromTable->axisY[0];
    int yMinValue = fromTable->axisY[fromTable->ySize-1];
    byte yMin = 0;
    byte yMax = 0;

    //If the requested Y value is greater/small than the maximum/minimum bin, reset Y to be that value
    if(Y > yMaxValue) { Y = yMaxValue; }
    if(Y < yMinValue) { Y = yMinValue; }

    //1st check is whether we're still in the same Y bin as last time
    if ( (Y >= fromTable->axisY[fromTable->lastYMax]) && (Y < fromTable->axisY[fromTable->lastYMin]) )
    {
      yMaxValue = fromTable->axisY[fromTable->lastYMax];
      yMinValue = fromTable->axisY[fromTable->lastYMin];
      yMax = fromTable->lastYMax;
      yMin = fromTable->lastYMin;
    }
    //2nd check is whether we're in the next MAP/TPS bin (Next one up)
    else if ( (fromTable->lastYMin > 0 ) && (Y <= fromTable->axisY[fromTable->lastYMin - 1 ]) && (Y > fromTable->axisY[fromTable->lastYMax - 1]) ) //First make sure we're not already at the top Y bin
    {
      yMax = fromTable->lastYMax - 1;
      fromTable->lastYMax = yMax;
      yMin = fromTable->lastYMin - 1;
      fromTable->lastYMin = yMin;
      yMaxValue = fromTable->axisY[fromTable->lastYMax];
      yMinValue = fromTable->axisY[fromTable->lastYMin];
    }
    //3rd check is to look at the previous bin (Next one down)
    else if ( ((fromTable->lastYMax + 1) < fromTable->ySize) && (Y <= fromTable->axisY[fromTable->lastYMin + 1]) && (Y > fromTable->axisY[fromTable->lastYMax + 1]) ) //First make sure we're not already at the bottom Y bin
    {
      yMax = fromTable->lastYMax + 1;
      fromTable->lastYMax = yMax;
      yMin = fromTable->lastYMin + 1;
      fromTable->lastYMin = yMin;
      yMaxValue = fromTable->axisY[fromTable->lastYMax];
      yMinValue = fromTable->axisY[fromTable->lastYMin];
    }
    else
    //If it's not caught by one of the above scenarios, give up and just run the loop
    {

      for (int8_t y = fromTable->ySize-1; y >= 0; y--)
      {
        //Checks the case where the Y value is exactly what was requested
        if ( (Y == fromTable->axisY[y]) || (y==0) )
        {
          yMaxValue = fromTable->axisY[y];
          yMinValue = fromTable->axisY[y];
          yMax = y;
          fromTable->lastYMax = yMax;
          yMin = y;
          fromTable->lastYMin = yMin;
          break;
        }
        //Normal case
        if ( (Y >= fromTable->axisY[y]) && (Y < fromTable->axisY[y-1]) )
        {
          yMaxValue = fromTable->axisY[y];
          yMinValue = fromTable->axisY[y-1];
          yMax = y;
          fromTable->lastYMax = yMax;
          yMin = y-1;
          fromTable->lastYMin = yMin;
          break;
        }
      }
    }


    /*
    At this point we have the 4 corners of the map where the interpolated value will fall in
    Eg: (yMin,xMin)  (yMin,xMax)

        (yMax,xMin)  (yMax,xMax)

    In the following calculation the table values are referred to by the following variables:
              A          B

              C          D

    */

    {
      //Create some normalised position values
      //These are essentially percentages (between 0 and 1) of where the desired value falls between the nearest bins on each axis


      //Initial check incase the values were hit straight on

      unsigned long p = (long)X - xMinValue;
      if (xMaxValue == xMinValue) { p = (p << TABLE_SHIFT_FACTOR); }  //This only occurs if the requested X value was equal to one of the X axis bins
      else { p = ( (p << TABLE_SHIFT_FACTOR) / (xMaxValue - xMinValue) ); } //This is the standard case

      unsigned long q;
      if (yMaxValue == yMinValue)
      {
        q = (long)Y - yMinValue;
        q = (q << TABLE_SHIFT_FACTOR);
      }
      //Standard case
      else
      {
        q = long(Y) - yMaxValue;
        q = TABLE_SHIFT_POWER - ( (q << TABLE_SHIFT_FACTOR) / (yMinValue - yMaxValue) );
      }

      uint32_t m = ((TABLE_SHIFT_POWER-p) * (TABLE_SHIFT_POWER-q)) >> TABLE_SHIFT_FACTOR;
      uint32_t n = (p * (TABLE_SHIFT_POWER-q)) >> TABLE_SHIFT_FACTOR;
      uint32_t o = ((TABLE_SHIFT_POWER-p) * q) >> TABLE_SHIFT_FACTOR;
      uint32_t r = (p * q) >> TABLE_SHIFT_FACTOR;

      uint16_t a = (fromTable->values[yMin][xMin] * ratio) / 128;
      uint16_t b = (fromTable->values[yMin][xMax] * ratio) / 128;
      uint16_t c = (fromTable->values[yMax][xMin] * ratio) / 128;
      uint16_t d = (fromTable->values[yMax][xMax] * ratio) / 128;

      fromTable->values[yMin][xMin] = (fromTable->values[yMin][xMin] * n + a * m) >> TABLE_SHIFT_FACTOR;
      fromTable->values[yMin][xMax] = (fromTable->values[yMin][xMax] * m + b * n) >> TABLE_SHIFT_FACTOR;
      fromTable->values[yMax][xMin] = (fromTable->values[yMax][xMin] * r + c * o) >> TABLE_SHIFT_FACTOR;
      fromTable->values[yMax][xMax] = (fromTable->values[yMax][xMax] * o + d * r) >> TABLE_SHIFT_FACTOR;

    }

    //Update the tables cache data
    fromTable->cacheIsValid = false;
}


void autotune()
{
    if (currentStatus.tpsDOT < 2)
    {
        uint16_t ratio = ((currentStatus.O2 * 128u) / currentStatus.afrTarget);

        if(ratio < 128)
        {
            uint8_t tmp = 128u - ratio;
            tmp = (tmp * 128u) / configPage9.autotuneStrength;
            ratio = 128 - tmp;
        }
        else if(ratio > 128)
        {
            uint8_t tmp = ratio - 128u;
            tmp = (tmp * 128u) / configPage9.autotuneStrength;
            ratio = tmp + 128;
        }
        
        if (ratio != 128)
            update3DTableValue(&fuelTable, currentStatus.fuelLoad, currentStatus.RPM, ratio);
    }
}