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
#include <boost/lexical_cast.hpp>

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/Worker.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Worker::Worker() {
}

Worker::~Worker() {  
}

Worker::Impl::Impl(const std::string& name, const WorkerOptions&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  name(name),
  expectedRate(0.0),
  callback(defaultOptions.callback),
  started(false),
  canceled(false),
  nodeImpl(nodeImpl) {
  std::string ns = std::string("workers/")+name;
  double rate = nodeImpl->getParam(ns+"/rate",
    1.0/defaultOptions.rate.expectedCycleTime().toSec());
  bool autostart = nodeImpl->getParam(ns+"/autostart",
    defaultOptions.autostart);
  expectedRate = ros::Rate(rate);
  
  ros::AdvertiseServiceOptions startOptions;
  startOptions.init<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    ns+"/start", boost::bind(&Worker::Impl::startCallback, this, _1, _2));
  startServer = nodeImpl->getNodeHandle().advertiseService(startOptions);
  
  ros::AdvertiseServiceOptions cancelOptions;
  cancelOptions.init<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    ns+"/cancel", boost::bind(&Worker::Impl::cancelCallback, this, _1, _2));
  cancelServer = nodeImpl->getNodeHandle().advertiseService(cancelOptions);
  
  ros::AdvertiseServiceOptions getStateOptions;
  getStateOptions.init<GetWorkerState::Request, GetWorkerState::Response>(
    ns+"/get_state", boost::bind(&Worker::Impl::getStateCallback, this,
    _1, _2));
  getStateServer = nodeImpl->getNodeHandle().advertiseService(getStateOptions);
  
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

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Worker::getName() const {
  if (impl)
    return impl->name;
  else
    return std::string();
}

bool Worker::Impl::isValid() const {
  return startServer && cancelServer && getStateServer;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Worker::shutdown() {
  if (impl)
    impl->unadvertise();
}

void Worker::Impl::unadvertise() {
  if (isValid()) {
    startServer.shutdown();
    cancelServer.shutdown();
    getStateServer.shutdown();
  }
}

void Worker::Impl::start() {
  boost::mutex::scoped_lock lock(mutex);

  if (!started) {
    started = true;
    canceled = false;
    
    timer.start();
  }
}

void Worker::Impl::cancel(bool block) {
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
    }
  }
}

void Worker::Impl::timerCallback(const ros::TimerEvent& timerEvent) {
  boost::mutex::scoped_lock lock(mutex);
  
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
  }
  else if (done) {
    timer.stop();
    started = false;
  }  
  
  threadId = boost::thread::id();
}

bool Worker::Impl::startCallback(std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  start();
  return true;
}

bool Worker::Impl::cancelCallback(std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
  cancel(false);
  return true;
}

bool Worker::Impl::getStateCallback(GetWorkerState::Request& request,
    GetWorkerState::Response& response) {
  boost::mutex::scoped_lock lock(mutex);
  
  response.started = started;
  response.active = (threadId != boost::thread::id());
  response.canceled = canceled;
  
  return true;
}

}
