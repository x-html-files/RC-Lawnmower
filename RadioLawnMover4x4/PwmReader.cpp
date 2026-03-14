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

void PwmReader::setLimits(int fromLow, int fromHigh, int center, int center_threshold)
{
  _fromLow = fromLow;
  _fromHigh = fromHigh;
  _centerFrom = center - center_threshold;
  _centerTo = center + center_threshold;
}

int PwmReader::bucketMap(int input, int inLow, int inHigh, int outLow, int outHigh) {
    int numBuckets = outHigh - outLow + 1;          // = 3 buckets (2, 3, 4)
    int inputRange = inHigh - inLow;
    int bucket = (long)(input - inLow) * numBuckets / (inputRange + 1);
    // Clamp to valid range
    if (bucket < 0) bucket = 0;
    if (bucket >= numBuckets) bucket = numBuckets - 1;
    return outLow + bucket;
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
    return bucketMap(v, _fromLow,  _centerFrom, _toLow, _toMiddle);
  }

  // lower middle to higher middle
  if( v <= _centerTo)
  {
    return _toMiddle;
  }
  
  // higher middle to highest
  return bucketMap(v, _centerTo,  _fromHigh, _toMiddle, _toHigh);
}
