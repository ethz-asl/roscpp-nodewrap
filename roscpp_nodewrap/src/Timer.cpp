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

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/timer/Timer.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Timer::Timer() {
}

Timer::Timer(const Timer& src) :
  impl(src.impl) {
}

Timer::Timer(const ros::TimerOptions& options, const NodeImplPtr& nodeImpl) :
  impl(new Impl(options, nodeImpl)) {
}

Timer::~Timer() {  
}

Timer::Impl::Impl(const ros::TimerOptions& options, const NodeImplPtr&
    nodeImpl) :
  started(false),
  handle(-1),
  period(options.period),
  autostart(options.autostart),
  oneshot(options.oneshot),
  callback(options.callback),
  callbackQueue(options.callback_queue),
  trackedObject(options.tracked_object),
  hasTrackedObject(options.tracked_object),
  nodeImpl(nodeImpl) {
}

Timer::Impl::~Impl() {
  stop();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Timer::setPeriod(const ros::Duration& period) {
  if (impl)
    impl->setPeriod(period);
}

bool Timer::hasPending() {
  if (impl)
    return impl->hasPending();
  else
    return false;
}

void Timer::Impl::setPeriod(const ros::Duration& period) {
  this->period = period;
  nodeImpl->timerManager.impl->setPeriod(handle, period);
}

bool Timer::Impl::hasPending() {
  if (isValid() && handle != -1)
    return  nodeImpl->timerManager.impl->hasPending(handle);
  else
    return false;
}

bool Timer::Impl::isValid() const {
  return !period.isZero();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Timer::start() {
  if (impl)
    impl->start();
}

void Timer::stop() {
  if (impl)
    impl->stop();
}

void Timer::Impl::start() {
  if (!started) {
    ros::VoidConstPtr trackedObject;
    if (hasTrackedObject)
      trackedObject = this->trackedObject.lock();

    handle = nodeImpl->timerManager.impl->addTimer(period, callback,
      callbackQueue, trackedObject, oneshot);
    
    started = true;
  }
}

void Timer::Impl::stop() {
  if (started) {
    started = false;
    nodeImpl->timerManager.impl->removeTimer(handle);
    handle = -1;
  }
}

}
