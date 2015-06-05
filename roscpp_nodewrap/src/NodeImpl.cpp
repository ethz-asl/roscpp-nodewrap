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

#include <roscpp_nodewrap/Signal.h>

#include "roscpp_nodewrap/NodeImpl.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

NodeImpl::NodeImpl() :
  nodelet(false) {
}

NodeImpl::~NodeImpl() {  
  Signal::unbind(SIGINT, &NodeImpl::shutdown, this);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const std::string& NodeImpl::getName() const {
  return name;
}

bool NodeImpl::isNodelet() const {
  return nodelet;
}

bool NodeImpl::isValid() const {
  return nodeHandle && nodeHandle->ok();
}

ros::NodeHandle& NodeImpl::getNodeHandle() const {
  return *nodeHandle;
}

void NodeImpl::setDiagnosticsHardwareId(const std::string& hardwareId) {
  if (!diagnosticUpdater)
    diagnosticUpdater = DiagnosticUpdater(shared_from_this());
  
  diagnosticUpdater.setHardwareId(hardwareId);
}

ros::AdvertiseOptions NodeImpl::getAdvertiseOptions(const std::string& ns,
    const ros::AdvertiseOptions& defaultOptions) const {
  ros::AdvertiseOptions options = defaultOptions;
  
  options.topic = getParam(ros::names::append(ns, "topic"),
    defaultOptions.topic);
  options.queue_size = getParam(ros::names::append(ns, "queue_size"),
    (int)defaultOptions.queue_size);
  options.latch = getParam(ros::names::append(ns, "latch"),
    defaultOptions.latch);
  
  return options;
}

ros::SubscribeOptions NodeImpl::getSubscribeOptions(const std::string& ns,
    const ros::SubscribeOptions& defaultOptions) const {
  ros::SubscribeOptions options = defaultOptions;
  
  options.topic = getParam(ros::names::append(ns, "topic"),
    defaultOptions.topic);
  options.queue_size = getParam(ros::names::append(ns, "queue_size"),
    (int)defaultOptions.queue_size);
  
  return options;
}

ros::AdvertiseServiceOptions NodeImpl::getAdvertiseServiceOptions(const
    std::string& ns, const ros::AdvertiseServiceOptions& defaultOptions)
    const {
  ros::AdvertiseServiceOptions options = defaultOptions;
  
  options.service = getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  
  return options;
}

ros::ServiceClientOptions NodeImpl::getServiceClientOptions(const std::string&
    ns, const ros::ServiceClientOptions& defaultOptions) const {
  ros::ServiceClientOptions options = defaultOptions;
  
  options.service = getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  options.persistent = getParam(ros::names::append(ns, "persistent"),
    defaultOptions.persistent);
  
  return options;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

Timer NodeImpl::createTimer(const ros::TimerOptions& options) {
  if (!timerManager)
    timerManager = TimerManager(shared_from_this());
  
  return timerManager.addTimer(options);
}

ros::Publisher NodeImpl::advertise(const std::string& name, const
    ros::AdvertiseOptions& defaultOptions) {
  ros::AdvertiseOptions options = getAdvertiseOptions(
    ros::names::append("publishers", name), defaultOptions);
  return this->getNodeHandle().advertise(options);
}

ros::Subscriber NodeImpl::subscribe(const std::string& name, const
    ros::SubscribeOptions& defaultOptions) {
  ros::SubscribeOptions options = getSubscribeOptions(
    ros::names::append("subscribers", name), defaultOptions);
  return this->getNodeHandle().subscribe(options);
}

ros::ServiceServer NodeImpl::advertiseService(const std::string& name,
    const ros::AdvertiseServiceOptions& defaultOptions) {
  ros::AdvertiseServiceOptions options = getAdvertiseServiceOptions(
    ros::names::append("servers", name), defaultOptions);
  return this->getNodeHandle().advertiseService(options);
}

ros::ServiceClient NodeImpl::serviceClient(const std::string& name,
    const ros::ServiceClientOptions& defaultOptions) {
  ros::ServiceClientOptions options = getServiceClientOptions(
    ros::names::append("clients", name), defaultOptions);
  return this->getNodeHandle().serviceClient(options);
}

Worker NodeImpl::addWorker(const std::string& name, const WorkerOptions&
    defaultOptions) {
  if (!workerManager)
    workerManager = WorkerManager(shared_from_this());
  
  return workerManager.addWorker(name, defaultOptions);
}

void NodeImpl::start(const std::string& name, bool nodelet, const
    ros::NodeHandlePtr& nodeHandle) {
  this->name = name;
  this->nodelet = nodelet;
  this->nodeHandle = nodeHandle;
  
  init();
  
  Signal::bind(SIGINT, &NodeImpl::shutdown, this);
}

void NodeImpl::shutdown() {
  if (diagnosticUpdater)
    diagnosticUpdater.shutdown();
  if (workerManager)
    workerManager.shutdown();
  
  cleanup();

  this->name.clear();
  this->nodelet = false;
  this->nodeHandle.reset();
}

}
