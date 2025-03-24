#ifndef PwmReader_h
#define PwmReader_h
#include <Ewma.h>

class PwmReader {
  public:
    PwmReader();
    PwmReader(float filterNumber);
    void setLimits(int lowest, int fromLow, int fromHigh, int center, int center_threshold);
    void setMapping(long toLow = 0, long toHigh = 255);    
    int mapTo(int v);
  private:
    int _pin;        
    Ewma adcFilter;
    int _lowest;
    int _fromHigh;
    int _fromLow;
    int _centerFrom;
    int _centerTo;
    int _toLow;
    int _toHigh;
    int _toMiddle;
};
#endif