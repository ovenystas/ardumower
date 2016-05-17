// running median filter
//
// usage:
//   RunningMedian<unsigned int,32> myMedian;
//   if (myMedian.getStatus() == myMedian.OK)  myMedian.getMedian(_median);

#ifndef RunningMedian_h
#define RunningMedian_h
//
//    FILE: RunningMedian.h
//  AUTHOR: Rob dot Tillaart at gmail dot com
// PURPOSE: RunningMedian library for Arduino
// VERSION: 0.2.00 - template edition
//     URL: http://arduino.cc/playground/Main/RunningMedian
// HISTORY: 0.2.00 first template version by Ronny
//          0.2.01 added getAverage(uint8_t nMedians, float val)
//
// Released to the public domain
//

#include <inttypes.h>

template<typename T, int N> class RunningMedian
{

  public:

    enum STATUS
    {
      OK = 0, NOK = 1
    };

    RunningMedian()
    {
      size = N;
      clear();
    }

    void clear()
    {
      cnt = 0;
      idx = 0;
    }

    void add(T value)
    {
      ar[idx++] = value;
      if (idx >= size)
      {
        idx = 0; // wrap around
      }
      if (cnt < size)
      {
        cnt++;
      }
    }

    STATUS getMedian(T& value)
    {
      if (cnt > 0)
      {
        sort();
        value = as[cnt / 2];
        return OK;
      }
      return NOK;
    }

    STATUS getAverage(float &value)
    {
      if (cnt > 0)
      {
        float sum = 0;
        for (uint8_t i = 0; i < cnt; i++)
        {
          sum += ar[i];
        }
        value = sum / cnt;
        return OK;
      }
      return NOK;
    }

    STATUS getAverage(uint8_t nMedians, float &value)
    {
      if ((cnt > 0) && (nMedians > 0))
      {
        if (cnt < nMedians)
        {
          nMedians = cnt;     // when filling the array for first time
        }
        uint8_t start = ((cnt - nMedians) / 2);
        uint8_t stop = start + nMedians;
        sort();
        float sum = 0;
        for (uint8_t i = start; i < stop; i++)
        {
          sum += as[i];
        }
        value = sum / nMedians;
        return OK;
      }
      return NOK;
    }

    STATUS getHighest(T& value)
    {
      if (cnt > 0)
      {
        sort();
        value = as[cnt - 1];
        return OK;
      }
      return NOK;
    }

    STATUS getLowest(T& value)
    {
      if (cnt > 0)
      {
        sort();
        value = as[0];
        return OK;
      }
      return NOK;
    }

    unsigned getSize()
    {
      return size;
    }

    unsigned getCount()
    {
      return cnt;
    }

    STATUS getStatus()
    {
      return (cnt > 0 ? OK : NOK);
    }

  private:
    uint8_t size;
    uint8_t cnt;
    uint8_t idx;
    T ar[N];
    T as[N];
    void sort()
    {
      // copy
      for (uint8_t i = 0; i < cnt; i++)
      {
        as[i] = ar[i];
      }

      // sort all
      for (uint8_t i = 0; i < cnt - 1; i++)
      {
        uint8_t m = i;
        for (uint8_t j = i + 1; j < cnt; j++)
        {
          if (as[j] < as[m])
          {
            m = j;
          }
        }
        if (m != i)
        {
          T t = as[m];
          as[m] = as[i];
          as[i] = t;
        }
      }
    }
};

#endif

