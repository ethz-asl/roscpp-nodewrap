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

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/worker/AsyncWorker.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

AsyncWorker::AsyncWorker() {
}

AsyncWorker::AsyncWorker(const AsyncWorker& src) :
  Worker(src) {
}

AsyncWorker::AsyncWorker(const std::string& name, const WorkerOptions&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  Worker(ImplPtr(new Impl(name, defaultOptions, nodeImpl))) {
}

AsyncWorker::~AsyncWorker() {  
}

AsyncWorker::Impl::Impl(const std::string& name, const WorkerOptions&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  Worker::Impl(name, defaultOptions, nodeImpl),
  resetTimer(true) {
  ros::TimerOptions timerOptions;
  
  timerOptions.period = ros::Duration(0.0);
  timerOptions.oneshot = expectedCycleTime.isZero();  
  timerOptions.autostart = false;
  timerOptions.callback = boost::bind(&AsyncWorker::Impl::timerCallback,
    this, _1);
  timerOptions.callback_queue = defaultOptions.callbackQueue;
  timerOptions.tracked_object = defaultOptions.trackedObject;
  
  timer = getNodeHandle().createTimer(timerOptions);
}

AsyncWorker::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void AsyncWorker::Impl::safeStart() {
  timer.start();
}

void AsyncWorker::Impl::safeWake() {
  resetTimer = true;
  timer.setPeriod(ros::Duration(0.0));
}

void AsyncWorker::Impl::safeStop() {
  timer.stop();
}

void AsyncWorker::Impl::timerCallback(const ros::TimerEvent& timerEvent) {
  if (resetTimer) {
    timer.setPeriod(expectedCycleTime);
    resetTimer = false;
  }

  runOnce();
}

}