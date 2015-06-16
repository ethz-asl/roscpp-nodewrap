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

#include "roscpp_nodewrap/worker/Worker.h"
#include "roscpp_nodewrap/worker/AsyncWorker.h"
#include "roscpp_nodewrap/worker/SyncWorker.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Worker::Worker() {
}

Worker::Worker(const Worker& src) :
  Managed<Worker, std::string>(src) {
}

Worker::~Worker() {  
}

Worker::Impl::Impl(const WorkerOptions& defaultOptions, const std::string&
    name, const ManagerImplPtr& manager) :
  Managed<Worker, std::string>::Impl(name, manager),
  autostart(true),
  callback(defaultOptions.callback),
  started(false),
  canceled(false) {
  std::string ns = defaultOptions.ns.empty() ?
    ros::names::append("workers", name) : defaultOptions.ns;
  
  double expectedFrequency = getNode()->getParam(
    ros::names::append(ns, "rate"), defaultOptions.frequency);
  expectedCycleTime = (expectedFrequency > 0.0) ?
    ros::Duration(1.0/expectedFrequency) : ros::Duration(0.0);
  autostart = getNode()->getParam(ros::names::append(ns, "autostart"),
    defaultOptions.autostart);
  
  ros::AdvertiseServiceOptions startOptions;
  startOptions.init<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    ros::names::append(ns, "start"),
    boost::bind(&Worker::Impl::startCallback, this, _1, _2));
  startServer = getNode()->advertiseService(
    ros::names::append(ns, "start"), startOptions);
  
  ros::AdvertiseServiceOptions cancelOptions;
  cancelOptions.init<std_srvs::Empty::Request, std_srvs::Empty::Response>(
    ros::names::append(ns, "cancel"),
    boost::bind(&Worker::Impl::cancelCallback, this, _1, _2));
  cancelServer = getNode()->advertiseService(
    ros::names::append(ns, "cancel"), cancelOptions);
  
  ros::AdvertiseServiceOptions getFrequencyOptions;
  getFrequencyOptions.init<GetWorkerFrequency::Request,
    GetWorkerFrequency::Response>(ros::names::append(ns, "get_frequency"),
    boost::bind(&Worker::Impl::getFrequencyCallback, this, _1, _2));
  getFrequencyServer = getNode()->advertiseService(
    ros::names::append(ns, "get_frequency"), getFrequencyOptions);
  
  ros::AdvertiseServiceOptions getStateOptions;
  getStateOptions.init<GetWorkerState::Request, GetWorkerState::Response>(
    ros::names::append(ns, "get_state"),
    boost::bind(&Worker::Impl::getStateCallback, this, _1, _2));
  getStateServer = getNode()->advertiseService(
    ros::names::append(ns, "get_state"), getStateOptions);
  
  FrequencyTaskOptions frequencyTaskOptions(
    defaultOptions.frequencyTaskOptions);
  frequencyTaskOptions.ns = ros::names::append(ns, "diagnostics/frequency");
  frequencyTaskOptions.expected = expectedFrequency;
  frequencyTask = getNode()->addDiagnosticTask<StatefulFrequencyTask>(
    std::string("Worker ")+name+" Frequency", frequencyTaskOptions);
}

Worker::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Worker::getName() const {
  return getIdentifier();
}

FrequencyStatistics::Estimates Worker::getStatisticsEstimates() const {
  if (impl)
    return impl->as<Worker::Impl>().getStatisticsEstimates();
  else
    return FrequencyStatistics::Estimates();
}

FrequencyStatistics::Estimates Worker::Impl::getStatisticsEstimates() const {
  boost::mutex::scoped_lock lock(mutex);
  
  return frequencyTask.getStatisticsEstimates();
}

bool Worker::Impl::isValid() const {
  return startServer && cancelServer && getFrequencyServer &&
    getStateServer && frequencyTask;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

Timer Worker::Impl::createTimer(const ros::TimerOptions& options) {
  return getNode()->createTimer(options);
}

void Worker::start() {
  if (impl)
    impl->as<Worker::Impl>().start();
}

void Worker::cancel(bool block) {
  if (impl)
    impl->as<Worker::Impl>().cancel(block);
}

void Worker::wake() {
  if (impl)
    impl->as<Worker::Impl>().wake();
}

void Worker::Impl::shutdown() {
  if (isValid()) {
    cancel(true);
    
    startServer.shutdown();
    cancelServer.shutdown();
    
    getFrequencyServer.shutdown();
    getStateServer.shutdown();
    
    frequencyTask.shutdown();
  }
}

void Worker::Impl::start() {
  boost::mutex::scoped_lock lock(mutex);

  if (!started) {
    started = true;
    canceled = false;
    
    startTime = ros::Time();
    timeOfLastCycle = ros::Time();
    
    safeStart();
  }
}

void Worker::Impl::wake() {
  boost::mutex::scoped_lock lock(mutex);

  if (started && (threadId == boost::thread::id()))
    safeWake();
}

void Worker::Impl::cancel(bool block) {
  boost::mutex::scoped_lock lock(mutex);
  
  if (started) {
    if ((threadId != boost::thread::id()) &&
        (threadId != boost::this_thread::get_id())) {
      canceled = true;
      if (block)
        cancelCondition.wait(lock);
    }
    else {
      safeStop();
      
      started = false;
      frequencyTask.disable();
      
      if (!startTime.isZero())
        NODEWRAP_MEMBER_INFO(
          "Worker [%s] has been canceled after %.3f second(s).",
          identifier.c_str(), (ros::Time::now()-startTime).toSec());
      else
        NODEWRAP_MEMBER_INFO(
          "Worker [%s] has been canceled.", identifier.c_str());
    }
  }
}

void Worker::Impl::runOnce() {
  boost::mutex::scoped_lock lock(mutex);
  
  if (startTime.isZero()) {
    startTime = ros::Time::now();
    NODEWRAP_MEMBER_INFO("Worker [%s] has been started.", identifier.c_str());
    
    frequencyTask.enable();
  }
  
  threadId = boost::this_thread::get_id();
  bool done = false;
  
  if (!canceled) {
    if (callback) {
      ros::Time timeOfCycle = ros::Time::now();
      if (timeOfLastCycle.isZero())
        actualCycleTime = expectedCycleTime;
      else
        actualCycleTime = timeOfCycle-timeOfLastCycle;
      timeOfLastCycle = timeOfCycle;
      
      frequencyTask.event(timeOfCycle);
      
      WorkerEvent workerEvent;
      workerEvent.expectedCycleTime = expectedCycleTime;
      workerEvent.actualCycleTime = actualCycleTime;
  
      {
        lock.unlock();
        done = !callback(workerEvent);
        lock.lock();  
      }
    }
  }
  
  if (canceled) {
    safeStop();
    cancelCondition.notify_all();
    
    started = false;
    canceled = false;
    frequencyTask.disable();
    
    NODEWRAP_MEMBER_INFO(
      "Worker [%s] has been canceled after %.3f second(s).",
      identifier.c_str(), (ros::Time::now()-startTime).toSec());
  }
  else if (done) {
    safeStop();
    
    started = false;
    frequencyTask.disable();
    
    NODEWRAP_MEMBER_INFO(
      "Worker [%s] has finished cleanly after %.3f second(s).",
      identifier.c_str(), (ros::Time::now()-startTime).toSec());
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

bool Worker::Impl::getFrequencyCallback(GetWorkerFrequency::Request& request,
    GetWorkerFrequency::Response& response) {
  boost::mutex::scoped_lock lock(mutex);

  frequencyTask.getStatisticsEstimates().toMessage(response.estimates);
  
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
