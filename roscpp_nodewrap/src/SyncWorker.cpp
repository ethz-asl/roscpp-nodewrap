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

#include "roscpp_nodewrap/SyncWorker.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

SyncWorker::SyncWorker() {
}

SyncWorker::SyncWorker(const SyncWorker& src) :
  Worker(src) {
}

SyncWorker::SyncWorker(const std::string& name, const WorkerOptions&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  impl(new Impl(name, defaultOptions, nodeImpl)) {
}

SyncWorker::~SyncWorker() {  
}

SyncWorker::Impl::Impl(const std::string& name, const WorkerOptions&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  Worker(name, defaultOptions, nodeImpl) {
  ros::TimerOptions timerOptions;
  timerOptions.period = (rate > 0.0) ? ros::Duration(1.0/rate) :
    ros::Duration();
  timerOptions.oneshot = (rate <= 0.0);
  timerOptions.autostart = false;
  timerOptions.callback = boost::bind(&Worker::Impl::timerCallback, this, _1);
  timerOptions.callback_queue = defaultOptions.callbackQueue;
  timerOptions.tracked_object = defaultOptions.trackedObject;
  timer = nodeImpl->getNodeHandle().createTimer(timerOptions);
  
  if (autostart)
    start();
}

SyncWorker::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void SyncWorker::Impl::start() {
  boost::mutex::scoped_lock lock(mutex);

  if (!started) {
    started = true;
    canceled = false;
    
    startTime = ros::Time();
    
    timer.start();
  }
}

void SyncWorker::Impl::cancel(bool block) {
  boost::mutex::scoped_lock lock(mutex);
  
  if (started) {
    if ((threadId != boost::thread::id()) &&
        (threadId != boost::this_thread::get_id())) {
      canceled = true;
      if (block)
        condition.wait(lock);
    }
    else {
      timer.stop();
      started = false;
      
      NODEWRAP_WORKER_INFO(
        "Worker [%s] has been canceled after %.3f second(s).",
        name.c_str(), (ros::Time::now()-startTime).toSec());
    }
  }
}

void SyncWorker::Impl::timerCallback(const ros::TimerEvent& timerEvent) {
  boost::mutex::scoped_lock lock(mutex);
  
  if (startTime.isZero()) {
    startTime = ros::Time::now();
    NODEWRAP_WORKER_INFO("Worker [%s] has been started.", name.c_str());
  }
  
  threadId = boost::this_thread::get_id();
  bool done = false;
  
  if (!canceled) {
    if (callback) {
      WorkerEvent workerEvent;

      workerEvent.expectedCycleTime = expectedRate.expectedCycleTime();
      workerEvent.timing = timerEvent;
  
      lock.unlock();
      done = !callback(workerEvent);
      lock.lock();  
    }
  }
  
  if (canceled) {
    timer.stop();
    condition.notify_all();
    
    started = false;
    canceled = false;
    
    NODEWRAP_WORKER_INFO(
      "Worker [%s] has been canceled after %.3f second(s).",
      name.c_str(), (ros::Time::now()-startTime).toSec());
  }
  else if (done) {
    timer.stop();
    started = false;
    
    NODEWRAP_WORKER_INFO(
      "Worker [%s] has finished cleanly after %.3f second(s).",
      name.c_str(), (ros::Time::now()-startTime).toSec());
  }
  
  threadId = boost::thread::id();
}

}
