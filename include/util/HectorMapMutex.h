#ifndef HECTOR_MAP_MUTEX_H_
#define HECTOR_MAP_MUTEX_H_

#include "util/MapLockerInterface.h"

#include <boost/thread/mutex.hpp>

class HectorMapMutex : public MapLockerInterface //纯虚函数
{
public:
  virtual void lockMap()
  {
    mapModifyMutex_.lock();
  }

  virtual void unlockMap()
  {
    mapModifyMutex_.unlock();
  }

  boost::mutex mapModifyMutex_;

};

#endif// HECTOR_MAP_MUTEX_H_
