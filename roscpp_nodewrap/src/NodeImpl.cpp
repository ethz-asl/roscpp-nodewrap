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

ros::AdvertiseOptions NodeImpl::getAdvertiseOptions(const std::string& key,
    const ros::AdvertiseOptions& defaultOptions) const {
  std::string ns = std::string("publishers/")+key;
  ros::AdvertiseOptions options = defaultOptions;
  
  options.topic = getParam(ns+"/topic", defaultOptions.topic);
  options.queue_size = getParam(ns+"/queue_size",
    (int)defaultOptions.queue_size);
  options.latch = getParam(ns+"/latch", defaultOptions.latch);
  
  return options;
}

ros::SubscribeOptions NodeImpl::getSubscribeOptions(const std::string& key,
    const ros::SubscribeOptions& defaultOptions) const {
  std::string ns = std::string("subscribers/")+key;
  ros::SubscribeOptions options = defaultOptions;
  
  options.topic = getParam(ns+"/topic", defaultOptions.topic);
  options.queue_size = getParam(ns+"/queue_size",
    (int)defaultOptions.queue_size);
  
  return options;
}

ros::AdvertiseServiceOptions NodeImpl::getAdvertiseServiceOptions(const
    std::string& key, const ros::AdvertiseServiceOptions& defaultOptions)
    const {
  std::string ns = std::string("servers/")+key;  
  ros::AdvertiseServiceOptions options = defaultOptions;
  
  options.service = getParam(ns+"/service", defaultOptions.service);
  
  return options;
}

ros::ServiceClientOptions NodeImpl::getServiceClientOptions(const std::string&
    key, const ros::ServiceClientOptions& defaultOptions) const {
  std::string ns = std::string("clients/")+key;  
  ros::ServiceClientOptions options = defaultOptions;
  
  options.service = getParam(ns+"/service", defaultOptions.service);
  options.persistent = getParam(ns+"/persistent", defaultOptions.persistent);
  
  return options;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

ros::Publisher NodeImpl::advertise(const std::string& param, const
    ros::AdvertiseOptions& defaultOptions) {
  ros::AdvertiseOptions options = getAdvertiseOptions(param, defaultOptions);
  return this->getNodeHandle().advertise(options);
}

ros::Subscriber NodeImpl::subscribe(const std::string& param, const
    ros::SubscribeOptions& defaultOptions) {
  ros::SubscribeOptions options = getSubscribeOptions(param, defaultOptions);
  return this->getNodeHandle().subscribe(options);
}

ros::ServiceServer NodeImpl::advertiseService(const std::string& param,
    const ros::AdvertiseServiceOptions& defaultOptions) {
  ros::AdvertiseServiceOptions options = getAdvertiseServiceOptions(
    param, defaultOptions);
  return this->getNodeHandle().advertiseService(options);
}

ros::ServiceClient NodeImpl::serviceClient(const std::string& name, const
    ros::ServiceClientOptions& defaultOptions) {
  ros::ServiceClientOptions options = getServiceClientOptions(name,
    defaultOptions);
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
  if (workerManager)
    workerManager.shutdown();
  
  cleanup();

  this->name.clear();
  this->nodelet = false;
  this->nodeHandle.reset();
}

}
