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
    param,const std::string& defaultTopic, uint32_t defaultQueueSize, bool
    defaultLatch) {
  std::string ns = std::string("publishers/")+param;
  std::string topic = getParam(ns+"/topic", defaultTopic);
  int queueSize = getParam(ns+"/queue_size", (int)defaultQueueSize);
  bool latch = getParam(ns+"/latch", defaultLatch);
  
  return this->getNodeHandle().advertise<M>(topic, queueSize, latch);
}

template <class M> ros::Publisher NodeImpl::advertise(const std::string& param,
    const std::string& defaultTopic, uint32_t defaultQueueSize, const
    ros::SubscriberStatusCallback& connectCallback, const
    ros::SubscriberStatusCallback& disconnectCallback, const ros::VoidConstPtr&
    trackedObject, bool defaultLatch) {
  std::string ns = std::string("publishers/")+param;
  std::string topic = getParam(ns+"/topic", defaultTopic);
  int queueSize = getParam(ns+"/queue_size", (int)defaultQueueSize);
  bool latch = getParam(ns+"/latch", defaultLatch);
  
  return this->getNodeHandle().advertise<M>(topic, queueSize, connectCallback,
    disconnectCallback, trackedObject, latch);
}

template <class M, class T> ros::Subscriber NodeImpl::subscribe(const
    std::string& param, const std::string& defaultTopic, uint32_t
    defaultQueueSize, void(T::*fp)(const boost::shared_ptr<const M>&),
    const ros::TransportHints& transportHints) {
  std::string ns = std::string("subscribers/")+param;
  std::string topic = getParam(ns+"/topic", defaultTopic);
  int queueSize = getParam(ns+"/queue_size", (int)defaultQueueSize);
  
  return this->getNodeHandle().subscribe(topic, queueSize, fp, (T*)this,
    transportHints);
}
  
template <class M, class T> ros::Subscriber NodeImpl::subscribe(const
    std::string& param, const std::string& defaultTopic, uint32_t
    defaultQueueSize, void(T::*fp)(const boost::shared_ptr<M const>&) const,
    const ros::TransportHints& transportHints) {
  std::string ns = std::string("subscribers/")+param;
  std::string topic = getParam(ns+"/topic", defaultTopic);
  int queueSize = getParam(ns+"/queue_size", (int)defaultQueueSize);
  
  return this->getNodeHandle().subscribe(topic, queueSize, fp, (T*)this,
    transportHints);
}

template <class MReq, class MRes, class T> ros::ServiceServer
    NodeImpl::advertiseService(const std::string& param, const std::string&
    defaultService, bool(T::*fp)(MReq&, MRes&), const ros::VoidConstPtr&
    trackedObject) {
  std::string ns = std::string("servers/")+param;
  std::string service = getParam(ns+"/service", defaultService);
  
  boost::function<bool (MReq&, MRes&)> f(boost::bind(fp, (T*)this, _1, _2));
  
  return this->getNodeHandle().advertiseService(service, f, trackedObject);
}

template <class MReq, class MRes, class T> ros::ServiceServer
    NodeImpl::advertiseService(const std::string& param, const std::string&
    defaultService, bool(T::*fp)(ros::ServiceEvent<MReq, MRes>&), const
    ros::VoidConstPtr& trackedObject) {
  std::string ns = std::string("servers/")+param;
  std::string service = getParam(ns+"/service", defaultService);
  
  boost::function<bool (ros::ServiceEvent<MReq, MRes>&)> f(
    boost::bind(fp, (T*)this, _1));
  
  return this->getNodeHandle().advertiseService(service, f, trackedObject);
}

template <class S> ros::ServiceClient NodeImpl::serviceClient(const
    std::string& param, const std::string& defaultService, bool
    defaultPersistent, const ros::M_string& headerValues) {
  std::string ns = std::string("clients/")+param;  
  std::string service = getParam(ns+"/service", defaultService);
  bool persistent = getParam(ns+"/persistent", defaultPersistent);
  
  return this->getNodeHandle().serviceClient<S>(service, persistent,
    headerValues);
}
    
}
