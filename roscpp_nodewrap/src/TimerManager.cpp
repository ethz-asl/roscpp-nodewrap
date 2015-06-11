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

#include <limits>

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
  Manager<Timer, int>(src) {
}

TimerManager::TimerManager(const NodeImplPtr& node) {
  impl.reset(new Impl(node));
}

TimerManager::~TimerManager() {  
}

TimerManager::Impl::Impl(const NodeImplPtr& node) :
  numTimers(0),
  newTimer(false),
  started(false),
  canceled(false),
  node(node) {
}

TimerManager::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const NodeImplPtr& TimerManager::Impl::getNode() const {
  return node;
}

void TimerManager::Impl::setTimerPeriod(int handle, const ros::Duration&
    period) {
  boost::mutex::scoped_lock lock(timerMutex);
  
  TimerInfoPtr timerInfo = findTimer(handle);

  if (timerInfo) {
    {
      boost::mutex::scoped_lock lock(waitingQueueMutex);
      
      timerInfo->period = period;
      timerInfo->expectedTimeOfNextCallback = ros::Time::now()+period;

      waitingQueue.sort(boost::bind(
        &TimerManager::Impl::compareWaitingTimers, this, _1, _2));
    }

    newTimer = true;
    timerCondition.notify_one();
  }
}

bool TimerManager::Impl::timerHasPending(int handle) {
  boost::mutex::scoped_lock lock(timerMutex);
  
  TimerInfoPtr timerInfo = findTimer(handle);

  if (timerInfo) {
    if (timerInfo->hasTrackedObject) {
      ros::VoidConstPtr trackedObject = timerInfo->trackedObject.lock();
      if (!trackedObject)
        return false;
    }

    boost::mutex::scoped_lock lock(timerInfo->mutex);
    
    return (timerInfo->expectedTimeOfNextCallback <= ros::Time::now()) ||
      timerInfo->numWaitingCallbacks;
  }
  else
    return false;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

Timer TimerManager::addTimer(const ros::TimerOptions& options) {
  Timer timer;
  
  if (impl) {
    {
      boost::mutex::scoped_lock lock(impl->mutex);
      
      int handle = ++impl->as<TimerManager::Impl>().numTimers;
      timer.impl.reset(new Timer::Impl(options, handle, impl));
      
      impl->instances.insert(std::make_pair(handle, timer.impl));
    }
    
    if (timer.impl->as<Timer::Impl>().autostart)
      timer.impl->as<Timer::Impl>().start();
  }
    
  return timer;
}

void TimerManager::Impl::startTimer(int handle) {
  Timer::ImplPtr timerImpl = find(handle);
  
  if (timerImpl) {
    TimerInfoPtr timerInfo(new TimerInfo);
    
    timerInfo->handle = handle;
    timerInfo->period = timerImpl->as<Timer::Impl>().period;
    timerInfo->callback = timerImpl->as<Timer::Impl>().callback;
    timerInfo->callbackQueue = timerImpl->as<Timer::Impl>().callbackQueue;
    timerInfo->expectedTimeOfLastCallback = ros::Time::now();
    timerInfo->expectedTimeOfNextCallback =
      timerInfo->expectedTimeOfLastCallback+timerInfo->period;
    timerInfo->oneshot = timerImpl->as<Timer::Impl>().oneshot;
    
    ros::VoidConstPtr trackedObject = 
      timerImpl->as<Timer::Impl>().trackedObject.lock();
      
    if (trackedObject) {
      timerInfo->trackedObject = timerImpl->as<Timer::Impl>().trackedObject;
      timerInfo->hasTrackedObject = true;
    }

    {
      boost::mutex::scoped_lock lock(timerMutex);
      timers.push_back(timerInfo);

      if (!started) {
        spinner = boost::thread(boost::bind(&TimerManager::Impl::spin, this));
        started = true;
      }

      {
        boost::mutex::scoped_lock lock(waitingQueueMutex);
        
        waitingQueue.push_back(timerInfo->handle);
        waitingQueue.sort(boost::bind(
          &TimerManager::Impl::compareWaitingTimers, this, _1, _2));
      }

      newTimer = true;
      timerCondition.notify_all();
    }
  }
}

void TimerManager::Impl::stopTimer(int handle) {
  ros::CallbackQueueInterface* callbackQueue = 0;
  uint64_t id = 0;

  {
    boost::mutex::scoped_lock lock(timerMutex);

    for (std::list<TimerInfoPtr>::iterator it = timers.begin();
        it != timers.end(); ++it) {
      const TimerInfoPtr& timerInfo = *it;
      
      if (timerInfo->handle == handle) {
        timerInfo->removed = true;
        callbackQueue = timerInfo->callbackQueue;
        id = (uint64_t)timerInfo.get();
        
        timers.erase(it);
        
        break;
      }
    }

    {
      boost::mutex::scoped_lock lock(waitingQueueMutex);

      std::list<int>::iterator it = std::find(waitingQueue.begin(),
        waitingQueue.end(), handle);
      
      if (it != waitingQueue.end())
        waitingQueue.erase(it);
    }
  }

  if (callbackQueue)
    callbackQueue->removeByID(id);
}

void TimerManager::Impl::shutdown() {
  canceled = true;
  
  {
    boost::mutex::scoped_lock lock(timerMutex);
    timerCondition.notify_all();
  }
  
  if (started)
    spinner.join();
}

bool TimerManager::Impl::compareWaitingTimers(int leftHandle, int
    rightHandle) {
  TimerInfoPtr leftTimerInfo = findTimer(leftHandle);
  TimerInfoPtr rightTimerInfo = findTimer(rightHandle);
  
  if (!leftTimerInfo || !rightTimerInfo)
    return leftTimerInfo < rightTimerInfo;

  return (leftTimerInfo->expectedTimeOfNextCallback <
    rightTimerInfo->expectedTimeOfNextCallback);
}

TimerInfoPtr TimerManager::Impl::findTimer(int handle) {
  for (std::list<TimerInfoPtr>::iterator it = timers.begin();
      it != timers.end(); ++it)
    if ((*it)->handle == handle)
      return *it;

  return TimerInfoPtr();
}

void TimerManager::Impl::scheduleTimerCallback(const TimerInfoPtr&
    timerInfo) {
  boost::mutex::scoped_lock lock(timerMutex);

  if (!timerInfo->removed) {
    updateNextTimerCallback(timerInfo, ros::Time::now());
    
    {
      boost::mutex::scoped_lock lock(waitingQueueMutex);

      waitingQueue.push_back(timerInfo->handle);
      waitingQueue.sort(boost::bind(
        &TimerManager::Impl::compareWaitingTimers, this, _1, _2));
    }

    newTimer = true;
    timerCondition.notify_one();
  }
}

void TimerManager::Impl::updateNextTimerCallback(const TimerInfoPtr&
    timerInfo, const ros::Time& now) {
  if (!timerInfo->oneshot) {
    if (timerInfo->expectedTimeOfNextCallback <= now) {
      timerInfo->expectedTimeOfLastCallback =
        timerInfo->expectedTimeOfNextCallback;
      timerInfo->expectedTimeOfNextCallback += timerInfo->period;
    }

    if (timerInfo->expectedTimeOfNextCallback+timerInfo->period < now)
      timerInfo->expectedTimeOfNextCallback = now;
  }
  else
    timerInfo->expectedTimeOfNextCallback = ros::Time(
      std::numeric_limits<int>::max(), 999999999);
}

void TimerManager::Impl::spin() {
  ros::Time now;
  
  while (!canceled) {
    ros::Time sleepEnd;

    boost::mutex::scoped_lock lock(timerMutex);

    if (ros::Time::now() < now) {
      now = ros::Time::now();

      for (std::list<TimerInfoPtr>::iterator it = timers.begin();
           it != timers.end(); ++it) {
        const TimerInfoPtr& timerInfo = *it;

        if (now < timerInfo->expectedTimeOfLastCallback) {
          timerInfo->expectedTimeOfLastCallback = now;
          timerInfo->expectedTimeOfNextCallback = now+timerInfo->period;
        }
      }
    }

    now = ros::Time::now();

    {
      boost::mutex::scoped_lock lock(waitingQueueMutex);

      if (!waitingQueue.empty()) {
        TimerInfoPtr timerInfo = findTimer(waitingQueue.front());

        while (!waitingQueue.empty() && timerInfo && 
            (timerInfo->expectedTimeOfNextCallback <= now)) {
          now = ros::Time::now();

          ros::CallbackInterfacePtr callback(
            new TimerQueueCallback(shared_from_this(), timerInfo,
              timerInfo->expectedTimeOfLastCallback,
              timerInfo->actualTimeOfLastCallback,
              timerInfo->expectedTimeOfNextCallback)
          );
          
          ros::CallbackQueueInterface* callbackQueue =
            timerInfo->callbackQueue ? timerInfo->callbackQueue :
            node->getNodeHandle().getCallbackQueue();
          callbackQueue->addCallback(callback, (uint64_t)timerInfo.get());

          waitingQueue.pop_front();
          if (waitingQueue.empty())
            break;

          timerInfo = findTimer(waitingQueue.front());
        }

        if (timerInfo)
          sleepEnd = timerInfo->expectedTimeOfNextCallback;
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
        int remainingUsecs = std::max((int)((sleepEnd-now).toSec()*1e6), 1);
        timerCondition.timed_wait(lock,
          boost::posix_time::microseconds(remainingUsecs));
      }
      else
        timerCondition.timed_wait(lock,
          boost::posix_time::milliseconds(1));
    }

    newTimer = false;
  }
}

}
