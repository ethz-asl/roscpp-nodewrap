/******************************************************************************
 * Copyright (C) 2014 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include <ros/callback_queue_interface.h>

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/timer/TimerInfo.h"
#include "roscpp_nodewrap/timer/TimerQueueCallback.h"

#include "roscpp_nodewrap/timer/TimerManager.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TimerManager::TimerManager() {
}

TimerManager::TimerManager(const TimerManager& src) :
  impl(src.impl) {
}

TimerManager::TimerManager(const NodeImplPtr& nodeImpl) :
  impl(new Impl(nodeImpl)) {
}

TimerManager::~TimerManager() {  
}

TimerManager::Impl::Impl(const NodeImplPtr& nodeImpl) :
  newTimer(false),
  numHandles(0),
  started(false),
  canceled(false),
  nodeImpl(nodeImpl) {
}

TimerManager::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void TimerManager::Impl::setPeriod(int timerHandle, const ros::Duration&
    period) {
  boost::mutex::scoped_lock timersLock(timersMutex);
  
  TimerInfoPtr timerInfo = findTimer(timerHandle);

  if (!timerInfo)
    return;
  
  {
    boost::mutex::scoped_lock waitingLock(waitingMutex);
    
    timerInfo->period = period;
    timerInfo->nextExpected = ros::Time::now()+period;

    waitingTimers.sort(boost::bind(&TimerManager::Impl::waitingCompare,
      this, _1, _2));
  }

  newTimer = true;
  timersCondition.notify_one();
}

bool TimerManager::Impl::hasPending(int timerHandle) {
  boost::mutex::scoped_lock timersLock(timersMutex);
  
  TimerInfoPtr timerInfo = findTimer(timerHandle);

  if (!timerInfo)
    return false;

  if (timerInfo->hasTrackedObject) {
    ros::VoidConstPtr trackedObject = timerInfo->trackedObject.lock();
    if (!trackedObject)
      return false;
  }

  boost::mutex::scoped_lock waitingLock(timerInfo->waitingMutex);

  return timerInfo->nextExpected <= ros::Time::now() ||
    timerInfo->waitingCallbacks;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void TimerManager::shutdown() {
  if (impl)
    impl->shutdown();
}

Timer TimerManager::addTimer(const ros::TimerOptions& options) {
  Timer timer;
  
  timer.impl.reset(new Timer::Impl(options, impl->nodeImpl));
  
  if (timer.impl->autostart)
    timer.impl->start();
    
  return timer;
}

int TimerManager::Impl::addTimer(const ros::Duration& period, const
    ros::TimerCallback& callback, ros::CallbackQueueInterface* callbackQueue,
    const ros::VoidConstPtr& trackedObject, bool oneshot) {
  TimerInfoPtr timerInfo(new TimerInfo);
  
  timerInfo->period = period;
  timerInfo->callback = callback;
  timerInfo->callbackQueue = callbackQueue;
  timerInfo->lastExpected = ros::Time::now();
  timerInfo->nextExpected = timerInfo->lastExpected+period;
  timerInfo->oneshot = oneshot;
  
  if (trackedObject) {
    timerInfo->trackedObject = trackedObject;
    timerInfo->hasTrackedObject = true;
  }

  {
    boost::mutex::scoped_lock handlesLock(handlesMutex);
    timerInfo->handle = numHandles++;
  }

  {
    boost::mutex::scoped_lock timersLock(timersMutex);
    timerInfos.push_back(timerInfo);

    if (!started) {
      spinner = boost::thread(boost::bind(&TimerManager::Impl::spin, this));
      started = true;
    }

    {
      boost::mutex::scoped_lock waitingLock(waitingMutex);
      
      waitingTimers.push_back(timerInfo->handle);
      waitingTimers.sort(boost::bind(&TimerManager::Impl::waitingCompare,
        this, _1, _2));
    }

    newTimer = true;
    timersCondition.notify_all();
  }

  return timerInfo->handle;
}

void TimerManager::Impl::removeTimer(int timerHandle) {
  ros::CallbackQueueInterface* callbackQueue = 0;
  uint64_t id = 0;

  {
    boost::mutex::scoped_lock timersLock(timersMutex);

    std::vector<TimerInfoPtr>::iterator it = timerInfos.begin();
    std::vector<TimerInfoPtr>::iterator end = timerInfos.end();
    
    for ( ; it != end; ++it) {
      const TimerInfoPtr& timerInfo = *it;
      
      if (timerInfo->handle == timerHandle) {
        timerInfo->removed = true;
        callbackQueue = timerInfo->callbackQueue;
        id = (uint64_t)timerInfo.get();
        
        timerInfos.erase(it);
        break;
      }
    }

    {
      boost::mutex::scoped_lock waitingLock(waitingMutex);

      std::list<int>::iterator it = std::find(waitingTimers.begin(),
        waitingTimers.end(), timerHandle);
      
      if (it != waitingTimers.end())
        waitingTimers.erase(it);
    }
  }

  if (callbackQueue)
    callbackQueue->removeByID(id);
}

void TimerManager::Impl::shutdown() {
  canceled = true;
  
  {
    boost::mutex::scoped_lock timersLock(timersMutex);
    timersCondition.notify_all();
  }
  
  if (started)
    spinner.join();
}

bool TimerManager::Impl::waitingCompare(int leftTimerHandle, int
    rightTimerHandle) {
  TimerInfoPtr leftTimerInfo = findTimer(leftTimerHandle);
  TimerInfoPtr rightTimerInfo = findTimer(rightTimerHandle);
  
  if (!leftTimerInfo || !rightTimerInfo)
    return leftTimerInfo < rightTimerInfo;

  return leftTimerInfo->nextExpected < rightTimerInfo->nextExpected;
}

TimerInfoPtr TimerManager::Impl::findTimer(int timerHandle) {
  std::vector<TimerInfoPtr>::iterator it = timerInfos.begin();
  std::vector<TimerInfoPtr>::iterator end = timerInfos.end();
  
  for ( ; it != end; ++it)
    if ((*it)->handle == timerHandle)
      return *it;

  return TimerInfoPtr();
}

void TimerManager::Impl::schedule(const TimerInfoPtr& timerInfo) {
  boost::mutex::scoped_lock timersLock(timersMutex);

  if (timerInfo->removed)
    return;

  updateNext(timerInfo, ros::Time::now());
  
  {
    boost::mutex::scoped_lock waitingLock(waitingMutex);

    waitingTimers.push_back(timerInfo->handle);
    waitingTimers.sort(boost::bind(&TimerManager::Impl::waitingCompare,
      this, _1, _2));
  }

  newTimer = true;
  timersCondition.notify_one();
}

void TimerManager::Impl::updateNext(const TimerInfoPtr& timerInfo, const
    ros::Time& now) {
  if (!timerInfo->oneshot) {
    if (timerInfo->nextExpected <= now) {
      timerInfo->lastExpected = timerInfo->nextExpected;
      timerInfo->nextExpected += timerInfo->period;
    }

    if (timerInfo->nextExpected+timerInfo->period < now)
      timerInfo->nextExpected = now;
  }
  else
    timerInfo->nextExpected = ros::Time(
      std::numeric_limits<int>::max(), 999999999);
}

void TimerManager::Impl::spin() {
  ros::Time now;
  
  while (!canceled) {
    ros::Time sleepEnd;

    boost::mutex::scoped_lock timersLock(timersMutex);

    if (ros::Time::now() < now) {
      now = ros::Time::now();

      std::vector<TimerInfoPtr>::iterator it = timerInfos.begin();
      std::vector<TimerInfoPtr>::iterator end = timerInfos.end();
      
      for ( ; it != end; ++it) {
        const TimerInfoPtr& timerInfo = *it;

        if (now < timerInfo->lastExpected) {
          timerInfo->lastExpected = now;
          timerInfo->lastExpected = now+timerInfo->period;
        }
      }
    }

    now = ros::Time::now();

    {
      boost::mutex::scoped_lock waitlock(waitingMutex);

      if (!waitingTimers.empty()) {
        TimerInfoPtr timerInfo = findTimer(waitingTimers.front());

        while (!waitingTimers.empty() && timerInfo && 
            (timerInfo->nextExpected <= now)) {
          now = ros::Time::now();

          ros::CallbackInterfacePtr callback(new TimerQueueCallback(
            shared_from_this(), timerInfo, timerInfo->lastExpected,
            timerInfo->lastActual, timerInfo->nextExpected));
          
          ros::CallbackQueueInterface* callbackQueue =
            timerInfo->callbackQueue ? timerInfo->callbackQueue :
            nodeImpl->getNodeHandle().getCallbackQueue();
          callbackQueue->addCallback(callback, (uint64_t)timerInfo.get());

          waitingTimers.pop_front();
          if (waitingTimers.empty())
            break;

          timerInfo = findTimer(waitingTimers.front());
        }

        if (timerInfo)
          sleepEnd = timerInfo->nextExpected;
      }
      else
        sleepEnd = now+ros::Duration(0.1);
    }

    while (!newTimer && (ros::Time::now() < sleepEnd) && !canceled) {
      if (ros::Time::now() < now)
        break;

      now = ros::Time::now();

      if (now >= sleepEnd)
        break;

      if (ros::Time::isSystemTime()) {
        int remainingUsecs = std::max((int)(
          (sleepEnd-now).toSec()*1e6), 1);
        timersCondition.timed_wait(timersLock,
          boost::posix_time::microseconds(remainingUsecs));
      }
      else
        timersCondition.timed_wait(timersLock,
          boost::posix_time::milliseconds(1));
    }

    newTimer = false;
  }
}

}
