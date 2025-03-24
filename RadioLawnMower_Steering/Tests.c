// // Online C compiler to run C program online
// #include <stdio.h>

// unsigned long timeout = 25000; // 25ms timeout


//     int _lowest;
//     int _fromHigh;
//     int _fromLow;
//     int _centerFrom;
//     int _centerTo;
//     int _toLow;
//     int _toHigh;
//     int _toMiddle;

// long map(long x, long in_min, long in_max, long out_min, long out_max) {
//       return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//     }
    
// void setMapping(long toLow, long toHigh)
// {
//   _toLow = toLow;
//   _toHigh = toHigh;
//   _toMiddle = (toHigh + toLow) / 2;
// }

// void setLimits(int lowest, int fromLow, int fromHigh, int center, int center_threshold)
// {
//   _lowest = lowest;
//   _fromLow = fromLow;
//   _fromHigh = fromHigh;
//   _centerFrom = center - center_threshold;
//   _centerTo = center + center_threshold;
// }

// int mapTo(int v)
// {
//   // v = adcFilter.filter(v); 

//   // below lower limit
//   if(v <= _fromLow)
//   {
//     return _toLow;
//   }
  
//   // over higher limit
//   if(v >= _fromHigh)
//   {
//     return _toHigh;
//   }

//   // from zero to lower middle
//   if(v <= _centerFrom)
//   {    
//     return map(v, _fromLow,  _centerFrom, _toLow, _toMiddle);
//   }

//   // lower middle to higher middle
//   if( v <= _centerTo)
//   {
//     return _toMiddle;
//   }
  
//   // higher middle to highest
//   return map(v, _centerTo,  _fromHigh, _toMiddle, _toHigh);
// }



// int main() {
    
//    setLimits(950, 950, 1920, 1450, 50);
//    setMapping(-50, 50);  // max speed 255
    
// printf("_lowest=%d\n",_lowest);
// printf("_fromHigh=%d\n",_fromHigh);
// printf("_fromLow=%d\n",_fromLow);
// printf("_centerFrom=%d\n",_centerFrom);
// printf("_centerTo=%d\n",_centerTo);
// printf("_toLow=%d\n",_toLow);
// printf("_toHigh=%d\n",_toHigh);
// printf("_toMiddle=%d\n",_toMiddle);
    
//     // Write C code here
//     int length =5;
//     int vs[] = {950, 1100, 1499, 1600, 1900};
//     int start =949;
//     for(int i=0;i<20;i++)
//     {
//     int v =start + i*50;
    
//     printf("%d->%d\n",v,  mapTo(v));
//     }

//     return 0;
// }