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

#include "roscpp_nodewrap/timer/TimerInfo.h"
#include "roscpp_nodewrap/timer/TimerManager.h"

#include "roscpp_nodewrap/timer/TimerQueueCallback.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TimerQueueCallback::TimerQueueCallback(const TimerManager::ImplPtr& manager,
    const TimerInfoPtr& timerInfo, const ros::Time& expectedTimeOfLastCallback,
    const ros::Time& actualTimeOfLastCallback, const ros::Time&
    expectedTimeOfCurrentCallback) :
  manager(manager),
  timerInfo(timerInfo),
  expectedTimeOfLastCallback(expectedTimeOfLastCallback),
  actualTimeOfLastCallback(actualTimeOfLastCallback),
  expectedTimeOfCurrentCallback(expectedTimeOfCurrentCallback),
  called(false) {
  boost::mutex::scoped_lock lock(timerInfo->mutex);
  ++timerInfo->numWaitingCallbacks;
}

TimerQueueCallback::~TimerQueueCallback() {  
  TimerInfoPtr timerInfo = this->timerInfo.lock();
  
  if (timerInfo) {
    boost::mutex::scoped_lock lock(timerInfo->mutex);
    --timerInfo->numWaitingCallbacks;
  }
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

ros::CallbackInterface::CallResult TimerQueueCallback::call() {
  TimerInfoPtr timerInfo = this->timerInfo.lock();
  
  if (!timerInfo)
    return Invalid;

  ++timerInfo->totalNumCalls;
  called = true;

  ros::VoidConstPtr trackedObject;
  if (timerInfo->hasTrackedObject) {
    trackedObject = timerInfo->trackedObject.lock();
    
    if (!trackedObject)
      return Invalid;
  }

  ros::TimerEvent event;
  
  event.last_expected = expectedTimeOfLastCallback;
  event.last_real = actualTimeOfLastCallback;
  event.current_expected = expectedTimeOfCurrentCallback;
  event.current_real = ros::Time::now();
  event.profile.last_duration = timerInfo->durationOfLastCallback;

  ros::WallTime callbackStart = ros::WallTime::now();
  timerInfo->callback(event);
  ros::WallTime callbackEnd = ros::WallTime::now();
  timerInfo->durationOfLastCallback = callbackEnd-callbackStart;

  timerInfo->actualTimeOfLastCallback = event.current_real;

  TimerManager::ImplPtr manager = this->manager.lock();
  if (!manager)
    return Invalid;
  
  manager->as<TimerManager::Impl>().scheduleTimerCallback(timerInfo);

  return Success;
}

}
