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

template <class T> Timer NodeImpl::createTimer(const ros::Duration& period,
    void(T::*callback)(const ros::TimerEvent&), bool oneshot, bool autostart) {
  ros::TimerOptions options;
  
  options.period = period;
  options.callback = boost::bind(callback, static_cast<T*>(this), _1);
  options.oneshot = oneshot;
  options.autostart = autostart;
  
  return this->createTimer(options);
}

template <class M> Publisher NodeImpl::advertise(const std::string& name,
    const std::string& defaultTopic, uint32_t defaultQueueSize, bool
    defaultLatch) {
  PublisherOptions defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize);
  defaultOptions.latch = defaultLatch;
  
  return this->advertise(name, defaultOptions);
}

template <class M> Publisher NodeImpl::advertise(const std::string& name,
    const std::string& defaultTopic, uint32_t defaultQueueSize, const
    ros::SubscriberStatusCallback& connectCallback, const
    ros::SubscriberStatusCallback& disconnectCallback, const ros::VoidConstPtr&
    trackedObject, bool defaultLatch) {
  PublisherOptions defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize,
    connectCallback, disconnectCallback);
  defaultOptions.latch = defaultLatch;
  defaultOptions.tracked_object = trackedObject;
  
  return this->advertise(name, defaultOptions);
}

template <class M, class T> Subscriber NodeImpl::subscribe(const
    std::string& name, const std::string& defaultTopic, uint32_t
    defaultQueueSize, void(T::*callback)(const boost::shared_ptr<const M>&),
    const ros::TransportHints& transportHints) {
  SubscriberOptions defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize,
    boost::bind(callback, static_cast<T*>(this), _1));
  defaultOptions.transport_hints = transportHints;
  
  return this->subscribe(name, defaultOptions);
}
  
template <class M, class T> Subscriber NodeImpl::subscribe(const
    std::string& name, const std::string& defaultTopic, uint32_t
    defaultQueueSize, void(T::*callback)(const boost::shared_ptr<M const>&)
    const, const ros::TransportHints& transportHints) {
  SubscriberOptions defaultOptions;
  
  defaultOptions.template init<M>(defaultTopic, defaultQueueSize,
    boost::bind(callback, (const T*)this, _1));
  defaultOptions.transport_hints = transportHints;
  
  return this->subscribe(name, defaultOptions);
}

template <class MReq, class MRes, class T> ServiceServer
    NodeImpl::advertiseService(const std::string& name, const std::string&
    defaultService, bool(T::*callback)(MReq&, MRes&), const ros::VoidConstPtr&
    trackedObject) {
  ServiceServerOptions defaultOptions;

  defaultOptions.template init<MReq, MRes>(defaultService,
    boost::bind(callback, static_cast<T*>(this), _1, _2));
  defaultOptions.tracked_object = trackedObject;
  
  return this->advertiseService(name, defaultOptions);
}

template <class S, class T> ServiceServer NodeImpl::advertiseService(
    const std::string& name, const std::string& defaultService,
    bool(T::*callback)(typename S::Request&, typename S::Response&),
    const ros::VoidConstPtr& trackedObject) {
  ServiceServerOptions defaultOptions;

  defaultOptions.template init<S>(defaultService,
    boost::bind(callback, static_cast<T*>(this), _1, _2));
  defaultOptions.tracked_object = trackedObject;
  
  return this->advertiseService(name, defaultOptions);
}

template <class MReq, class MRes> ServiceClient NodeImpl::serviceClient(
    const std::string& name, const std::string& defaultService, bool
    defaultPersistent, const ros::M_string& headerValues) {
  ServiceClientOptions defaultOptions;
  
  defaultOptions.template init<MReq, MRes>(defaultService, defaultPersistent,
    headerValues);
  
  return this->serviceClient(name, defaultOptions);
}

template <class S> ServiceClient NodeImpl::serviceClient(const
    std::string& name, const std::string& defaultService, bool
    defaultPersistent, const ros::M_string& headerValues) {
  ServiceClientOptions defaultOptions;
  
  defaultOptions.template init<S>(defaultService, defaultPersistent,
    headerValues);
  
  return this->serviceClient(name, defaultOptions);
}

template <class T> Worker NodeImpl::addWorker(const std::string& name,
    double defaultFrequency, bool(T::*callback)(const WorkerEvent&),
    bool defaultAutostart, bool synchronous) {
  WorkerOptions defaultOptions;
  
  defaultOptions.frequency = defaultFrequency;
  defaultOptions.callback = boost::bind(callback, static_cast<T*>(this), _1);
  defaultOptions.autostart = defaultAutostart;
  defaultOptions.synchronous = synchronous;
  
  return this->addWorker(name, defaultOptions);
}

template <class T> T NodeImpl::addDiagnosticTask(const std::string& name,
    const typename T::Options& defaultOptions) {
  if (!this->diagnosticUpdater)
    this->diagnosticUpdater = DiagnosticUpdater(shared_from_this());
  
  return this->diagnosticUpdater.template addTask<T>(name, defaultOptions);
}

template <class T> FunctionTask NodeImpl::addDiagnosticTask(const std::string&
    name, void(T::*callback)(diagnostic_updater::DiagnosticStatusWrapper&)
    const) {
  FunctionTaskOptions options;
  
  options.callback = boost::bind(callback, static_cast<T*>(this), _1);;
  
  return this->template addDiagnosticTask<FunctionTask>(name, options);
}


}
