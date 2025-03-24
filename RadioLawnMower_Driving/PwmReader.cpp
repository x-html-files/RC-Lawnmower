#include <Arduino.h>
#include "PwmReader.h"
#include <Ewma.h>

unsigned long timeout = 25000; // 25ms timeout

PwmReader::PwmReader() : adcFilter(1.0)
{

}

PwmReader::PwmReader(float filter) : adcFilter(filter)
{

}

void PwmReader::setMapping(long toLow = 0, long toHigh = 255)
{
  _toLow = toLow;
  _toHigh = toHigh;
  _toMiddle = (toHigh + toLow) / 2;
}

void PwmReader::setLimits(int lowest, int fromLow, int fromHigh, int center, int center_threshold)
{
  _lowest = lowest;
  _fromLow = fromLow;
  _fromHigh = fromHigh;
  _centerFrom = center - center_threshold;
  _centerTo = center + center_threshold;
}

int PwmReader::mapTo(int v)
{
  v = adcFilter.filter(v); 

  // below lower limit
  if(v <= _fromLow)
  {
    return _toLow;
  }
  
  // over higher limit
  if(v >= _fromHigh)
  {
    return _toHigh;
  }

  // from zero to lower middle
  if(v <= _centerFrom)
  {    
    return map(v, _fromLow,  _centerFrom, _toLow, _toMiddle);
  }

  // lower middle to higher middle
  if( v <= _centerTo)
  {
    return _toMiddle;
  }
  
  // higher middle to highest
  return map(v, _centerTo,  _fromHigh, _toMiddle, _toHigh);
}
