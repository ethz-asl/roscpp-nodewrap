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

using namespace roscpp_nodewrap;
  
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
  std::string ns = ros::names::append("publishers", key);
  ros::AdvertiseOptions options = defaultOptions;
  
  options.topic = getParam(ros::names::append(ns, "topic"),
    defaultOptions.topic);
  options.queue_size = getParam(ros::names::append(ns, "queue_size"),
    (int)defaultOptions.queue_size);
  options.latch = getParam(ros::names::append(ns, "latch"),
    defaultOptions.latch);
  
  return options;
}

ros::SubscribeOptions NodeImpl::getSubscribeOptions(const std::string& key,
    const ros::SubscribeOptions& defaultOptions) const {
  std::string ns = ros::names::append("subscribers", key);
  ros::SubscribeOptions options = defaultOptions;
  
  options.topic = getParam(ros::names::append(ns, "topic"),
    defaultOptions.topic);
  options.queue_size = getParam(ros::names::append(ns, "queue_size"),
    (int)defaultOptions.queue_size);
  
  return options;
}

ros::AdvertiseServiceOptions NodeImpl::getAdvertiseServiceOptions(const
    std::string& key, const ros::AdvertiseServiceOptions& defaultOptions)
    const {
  std::string ns = ros::names::append("servers", key);  
  ros::AdvertiseServiceOptions options = defaultOptions;
  
  options.service = getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  
  return options;
}

ros::ServiceClientOptions NodeImpl::getServiceClientOptions(const std::string&
    key, const ros::ServiceClientOptions& defaultOptions) const {
  std::string ns = ros::names::append("clients", key);
  ros::ServiceClientOptions options = defaultOptions;
  
  options.service = getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  options.persistent = getParam(ros::names::append(ns, "persistent"),
    defaultOptions.persistent);
  
  return options;
}

AdvertiseConfigOptions NodeImpl::getAdvertiseConfigOptions(const std::string&
    key, const AdvertiseConfigOptions& defaultOptions) const {
  std::string ns = ros::names::append("servers", key);  
  AdvertiseConfigOptions options = defaultOptions;
  
  options.service = getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  
  return options;
}

ConfigClientOptions NodeImpl::getConfigClientOptions(const std::string& key,
    const ConfigClientOptions& defaultOptions) const {
  std::string ns = ros::names::append("clients", key);
  ConfigClientOptions options = defaultOptions;
  
  options.service = getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  options.persistent = getParam(ros::names::append(ns, "persistent"),
    defaultOptions.persistent);
  
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

ros::ServiceClient NodeImpl::serviceClient(const std::string& param, const
    ros::ServiceClientOptions& defaultOptions) {
  ros::ServiceClientOptions options = getServiceClientOptions(param,
    defaultOptions);
  return this->getNodeHandle().serviceClient(options);
}

ConfigServer NodeImpl::advertiseConfig(const std::string& param, const
    std::string& defaultService) {
  AdvertiseConfigOptions options, defaultOptions;
  
  defaultOptions.service = defaultService.empty() ?
    getNodeHandle().getNamespace() : defaultService;
  options = getAdvertiseConfigOptions(param, defaultOptions);
  
  return ConfigServer(options, shared_from_this());
}

ConfigServer NodeImpl::advertiseConfig(const std::string& param, const
    AdvertiseConfigOptions& defaultOptions) {
  AdvertiseConfigOptions options = getAdvertiseConfigOptions(param,
    defaultOptions);
  return ConfigServer(options, shared_from_this());
}

ParamServer NodeImpl::advertiseParam(const AdvertiseParamOptions& options) {
  return ParamServer(options, shared_from_this());
}

ConfigClient NodeImpl::configClient(const std::string& param, const
    std::string& defaultService, bool defaultPersistent) {
  ConfigClientOptions options, defaultOptions;
  
  defaultOptions.service = defaultService.empty() ?
    getNodeHandle().getNamespace() : defaultService;
  defaultOptions.persistent = defaultPersistent;
  options = getConfigClientOptions(param, defaultOptions);
  
  return ConfigClient(options, shared_from_this());
}

ConfigClient NodeImpl::configClient(const std::string& param, const
    ConfigClientOptions& defaultOptions) {
  ConfigClientOptions options = getConfigClientOptions(param, defaultOptions);
  return ConfigClient(options, shared_from_this());
}

ParamClient NodeImpl::paramClient(const ParamClientOptions& options) {
  return ParamClient(options, shared_from_this());
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
  cleanup();
  
  this->name.clear();
  this->nodelet = false;
  this->nodeHandle.reset();
}

}
