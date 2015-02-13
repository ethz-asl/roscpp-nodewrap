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

namespace nodewrap {

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void NodeImpl::setParam(const std::string& key, const
    T& value) {
  this->getNodeHandle().setParam(key, value);
}
    
template <typename T> T NodeImpl::getParam(const std::string& key,
    const T& defaultValue) const {
  T value;
  this->getNodeHandle().param(key, value, defaultValue);
  
  return value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class M> ros::Publisher NodeImpl::advertise(const std::string&
    param, const std::string& defaultTopic, uint32_t defaultQueueSize, bool
    defaultLatch) {
  ros::AdvertiseOptions options, defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize);
  defaultOptions.latch = defaultLatch;
  options = getAdvertiseOptions(param, defaultOptions);
  
  return this->getNodeHandle().advertise(options);
}

template <class M> ros::Publisher NodeImpl::advertise(const std::string& param,
    const std::string& defaultTopic, uint32_t defaultQueueSize, const
    ros::SubscriberStatusCallback& connectCallback, const
    ros::SubscriberStatusCallback& disconnectCallback, const ros::VoidConstPtr&
    trackedObject, bool defaultLatch) {
  ros::AdvertiseOptions options, defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize,
    connectCallback, disconnectCallback);
  defaultOptions.latch = defaultLatch;
  defaultOptions.tracked_object = trackedObject;
  options = getAdvertiseOptions(param, defaultOptions);
  
  return this->getNodeHandle().advertise(options);
}

template <class M, class T> ros::Subscriber NodeImpl::subscribe(const
    std::string& param, const std::string& defaultTopic, uint32_t
    defaultQueueSize, void(T::*fp)(const boost::shared_ptr<const M>&),
    const ros::TransportHints& transportHints) {
  ros::SubscribeOptions options, defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize,
    boost::bind(fp, (T*)this, _1));
  defaultOptions.transport_hints = transportHints;
  options = getSubscribeOptions(param, defaultOptions);
  
  return this->getNodeHandle().subscribe(options);
}
  
template <class M, class T> ros::Subscriber NodeImpl::subscribe(const
    std::string& param, const std::string& defaultTopic, uint32_t
    defaultQueueSize, void(T::*fp)(const boost::shared_ptr<M const>&) const,
    const ros::TransportHints& transportHints) {
  ros::SubscribeOptions options, defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize,
    boost::bind(fp, (const T*)this, _1));
  defaultOptions.transport_hints = transportHints;
  options = getSubscribeOptions(param, defaultOptions);
  
  return this->getNodeHandle().subscribe(options);
}

template <class MReq, class MRes, class T> ros::ServiceServer
    NodeImpl::advertiseService(const std::string& param, const std::string&
    defaultService, bool(T::*fp)(MReq&, MRes&), const ros::VoidConstPtr&
    trackedObject) {
  ros::AdvertiseServiceOptions options, defaultOptions;

  defaultOptions.template init<MReq, MRes>(defaultService,
    boost::bind(fp, (T*)this, _1, _2));
  defaultOptions.tracked_object = trackedObject;
  options = getAdvertiseServiceOptions(param, defaultOptions);
  
  return this->getNodeHandle().advertiseService(options);
}

template <class S, class T> ros::ServiceServer NodeImpl::advertiseService(
    const std::string& param, const std::string& defaultService,
    bool(T::*fp)(S&), const ros::VoidConstPtr& trackedObject) {
  ros::AdvertiseServiceOptions options, defaultOptions;

  defaultOptions.template initBySpecType<S>(defaultService,
    boost::bind(fp, (T*)this, _1));
  defaultOptions.tracked_object = trackedObject;
  options = getAdvertiseServiceOptions(param, defaultOptions);
  
  return this->getNodeHandle().advertiseService(options);
}

template <class MReq, class MRes> ros::ServiceClient NodeImpl::serviceClient(
    const std::string& param, const std::string& defaultService, bool
    defaultPersistent, const ros::M_string& headerValues) {
  ros::ServiceClientOptions options, defaultOptions;
  
  defaultOptions.template init<MReq, MRes>(defaultService, defaultPersistent,
    headerValues);
  options = getServiceClientOptions(param, defaultOptions);
  
  return this->getNodeHandle().serviceClient(options);
}

template <class S> ros::ServiceClient NodeImpl::serviceClient(const
    std::string& param, const std::string& defaultService, bool
    defaultPersistent, const ros::M_string& headerValues) {
  ros::ServiceClientOptions options, defaultOptions;
  
  defaultOptions.template init<S>(defaultService, defaultPersistent,
    headerValues);
  options = getServiceClientOptions(param, defaultOptions);
  
  return this->getNodeHandle().serviceClient(options);
}

template <typename P> ParamServer NodeImpl::advertiseParam(const std::string&
    name, const std::string& service, bool cached) {
  AdvertiseParamOptions options;
  options.template init<P>(service.empty() ? ros::names::append(
    "params", name) : service, name, cached);
  
  return this->advertiseParam(options);
}

template <typename P> ParamClient NodeImpl::paramClient(const std::string&
    service, bool persistent) {
  ParamClientOptions options;
  options.template init<P>(service, persistent);
  
  return this->paramClient(options);
}

}
