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

#include "roscpp_nodewrap/timer/TimerManager.h"

#include "roscpp_nodewrap/timer/Timer.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Timer::Timer() {
}

Timer::Timer(const Timer& src) :
  Managed<Timer, int>(src) {
}

Timer::~Timer() {  
}

Timer::Impl::Impl(const ros::TimerOptions& options, int handle, const
    ManagerImplPtr& manager) :
  Managed<Timer, int>::Impl(handle, manager),
  started(false),
  period(options.period),
  autostart(options.autostart),
  oneshot(options.oneshot),
  callback(options.callback),
  callbackQueue(options.callback_queue),
  trackedObject(options.tracked_object),
  hasTrackedObject(options.tracked_object) {
}

Timer::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Timer::setPeriod(const ros::Duration& period) {
  if (impl)
    impl->as<Timer::Impl>().setPeriod(period);
}

bool Timer::hasPending() {
  if (impl)
    return impl->as<Timer::Impl>().hasPending();
  else
    return false;
}

void Timer::Impl::setPeriod(const ros::Duration& period) {
  this->period = period;
  manager->as<TimerManager::Impl>().setTimerPeriod(identifier, period);
}

bool Timer::Impl::hasPending() {
  if (isValid() && started)
    return manager->as<TimerManager::Impl>().timerHasPending(identifier);
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
    impl->as<Timer::Impl>().start();
}

void Timer::stop() {
  if (impl)
    impl->as<Timer::Impl>().stop();
}

void Timer::Impl::start() {
  if (!started) {
    manager->as<TimerManager::Impl>().startTimer(identifier);
    
    started = true;
  }
}

void Timer::Impl::stop() {
  if (started) {
    started = false;
    
    manager->as<TimerManager::Impl>().stopTimer(identifier);
  }
}

void Timer::Impl::shutdown() {
  stop();
}

}
