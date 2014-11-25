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

/** \file NodeImpl.h
  * \brief Header file providing the NodeImpl class interface
  */

#ifndef ROSCPP_NODEWRAP_NODEIMPL_H
#define ROSCPP_NODEWRAP_NODEIMPL_H

#include <ros/console.h>

#include <roscpp_nodewrap/NodeInterface.h>

namespace nodewrap {
  /** \brief Abstract class implementation of a ROS node(let)
    * 
    * This class provides the abstract, opaque implementation of a ROS
    * node(let). It should be inherited and implemented by all node wrappers
    * which are intended to become instantiable as both, ROS nodes and ROS
    * nodelets.
    */

  class NodeImpl :
    public virtual NodeInterface {
  public:
    /** \brief Default constructor
      * 
      * \note No constructor overloading is provided to allow for
      *   construction when dynamically loaded as nodelet.
      */
    NodeImpl();
    
    /** \brief Destructor
      */
    virtual ~NodeImpl();
  protected:
    /** \brief Perform node initialization
      * 
      * Here, all initialization code should be placed in the implementation,
      * including topic advertisement and subscription, service connection,
      * and parameter retrieval.
      * 
      * \note This function must be implemented by the subclass.
      */
    virtual void init() = 0;

    /** \brief Set a parameter value on the parameter server
      * 
      * \param[in] key The key referring to the parameter whose value
      *   shall be set.
      * \param[in] value The new value of the parameter.
      * 
      * \note If the parameter has already been defined, its value will
      *   be modified. Otherwhise, a new parameter will be created with
      *   the specified key and value.
      * 
      * \see ros::NodeHandle::getParam
      */
    template <typename T> void setParam(const std::string& key,
      const T& value);

    /** \brief Retrieve a parameter value from the parameter server
      * 
      * \param[in] key The key referring to the parameter whose value
      *   shall be retrieved.
      * \param[in] defaultValue The default value of the parameter.
      * \return The actual value of the parameter if the parameter
      *   has been defined or the default value otherwise.
      * 
      * \see ros::NodeHandle::setParam
      */
    template <typename T> T getParam(const std::string& key, const T&
      defaultValue) const;
    
    /** \brief Advertise a topic, with standard options
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new publisher.
      * \param[in] defaultTopic The default topic to advertise on.
      * \param[in] defaultQueueSize The default maximum number of outgoing
      *   messages to be queued for delivery to subscribers.
      * \param[in] defaultLatch If true, the last message published on
      *   this topic will by default be saved and sent to new subscribers
      *   when they connect.
      * \return On success, a ROS publisher that, when it goes out of scope,
      * will automatically release a reference on this advertisement. On
      * failure, an empty ROS publisher.
      * 
      * \see advertise(const std::string&, const ros::AdvertiseOptions&) for
      *   a detailed description of the method's behavior.
      */
    template <class M> ros::Publisher advertise(const std::string& param,
      const std::string& defaultTopic, uint32_t defaultQueueSize, bool
      defaultLatch = false);
    
    /** \brief Advertise a topic, with most of the avaiable options
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new publisher.
      * \param[in] defaultTopic The default topic to advertise on.
      * \param[in] defaultQueueSize The default maximum number of outgoing
      *   messages to be queued for delivery to subscribers.
      * \param[in] connectCallback The function to call when a subscriber
      *   connects.
      * \param[in] disconnectCallback The function to call when a subscriber
      *   disconnects.
      * \param[in] trackedObject A shared pointer to an object to track for
      *   these callbacks. If set, a weak pointer will be created to this
      *   object, and if the reference count goes to zero the subscriber
      *   callbacks will not get called. Note that setting this will cause
      *   a new reference to be added to the object before the callback,
      *   and for it to go out of scope (and potentially be deleted) in the
      *   code path (and therefore thread) that the callback is invoked from. 
      * \param[in] defaultLatch If true, the last message published on
      *   this topic will by default be saved and sent to new subscribers
      *   when they connect.
      * 
      * \see advertise(const std::string&, const ros::AdvertiseOptions&) for
      *   a detailed description of the method's behavior.
      */
    template <class M> ros::Publisher advertise(const std::string& param,
      const std::string& defaultTopic, uint32_t defaultQueueSize, const
      ros::SubscriberStatusCallback& connectCallback, const
      ros::SubscriberStatusCallback& disconnectCallback = 
      ros::SubscriberStatusCallback(), const ros::VoidConstPtr&
      trackedObject = ros::VoidConstPtr(), bool defaultLatch = false);
    
    /** \brief Advertise a topic, with full range of options
      * 
      * Extending the standard ROS interface for advertising topics, this
      * implementation takes a parameter name under which the publisher
      * configuration may be retrieved from the parameter server. In
      * particular, this parameter is expected to represent a structure of
      * the following YAML format:
      * 
      * - publishers:
      *   - param:
      *     - topic: <string>
      *     - queue_size: <int>
      *     - latch: <bool>
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new publisher.
      * \param[in] defaultOptions The default advertise options to use.
      * \return On success, a ROS publisher that, when it goes out of scope,
      * will automatically release a reference on this advertisement. On
      * failure, an empty ROS publisher.
      * 
      * \see ros::NodeHandle::advertise
      */
    ros::Publisher advertise(const std::string& param, const
      ros::AdvertiseOptions& defaultOptions);
    
    /** \brief Subscribe to a topic, with standard options (non-const version)
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new subscriber.
      * \param[in] defaultTopic The default topic to subscribe to.
      * \param[in] defaultQueueSize The default number of incoming messages
      *   to queue up for processing (messages in excess of this queue
      *   capacity will be discarded). 
      * \param[in] fp A member function pointer to call when a message has
      *   arrived.
      * \param[in] transportHints A transport hints structure which defines
      *   various transport-related options.
      * \return On success, a ROS subscriber that, when all copies of it go
      *   out of scope, will unsubscribe from this topic.
      * 
      * \note For portability between nodes and nodelets, the provided
      *   callback may only use the shared pointer signature.
      * 
      * \see subscribe(const std::string&, const ros::SubscribeOptions&) for
      *   a detailed description of the method's behavior.
      */
    template <class M, class T> ros::Subscriber subscribe(const std::string& 
      param, const std::string& defaultTopic, uint32_t defaultQueueSize,
      void(T::*fp)(const boost::shared_ptr<M const>&), const
      ros::TransportHints& transportHints = ros::TransportHints());  
    
    /** \brief Subscribe to a topic, with standard options (const version)
      * 
      * This is the const version of a subscription with standard options.
      * It differs from the above function only in the constness of the
      * provided callback function.
      * 
      * \note For portability between nodes and nodelets, the provided
      *   callback may only use the shared pointer signature.
      * 
      * \see subscribe(const std::string&, const ros::SubscribeOptions&) for
      *   a detailed description of the method's behavior.
      */
    template <class M, class T> ros::Subscriber subscribe(const std::string& 
      param, const std::string& defaultTopic, uint32_t defaultQueueSize,
      void(T::*fp)(const boost::shared_ptr<M const>&) const,
      const ros::TransportHints& transportHints = ros::TransportHints());
    
    /** \brief Subscribe to a topic, with full range of options
      * 
      * Extending the standard ROS interface for subscribing to topics, this
      * implementation takes a parameter name under which the subscribe
      * configuration may be retrieved from the parameter server. In
      * particular, this parameter is expected to represent a structure of
      * the following YAML format:
      * 
      * - subscribers:
      *   - param:
      *     - topic: <string>
      *     - queue_size: <int>
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new subscriber.
      * \param[in] defaultOptions The default subscribe options to use.
      * \return On success, a ROS subscriber that, when all copies of it go
      *   out of scope, will unsubscribe from this topic.
      * 
      * \see ros::NodeHandle::subscribe
      */
    ros::Subscriber subscribe(const std::string& param, const
      ros::SubscribeOptions& defaultOptions);
    
    /** \brief Advertise a service, with standard options
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new service server.
      * \param[in] defaultService The default service name to advertise on.
      * \param[in] callback The callback to call when the service is called.
      * \param[in] trackedObject A shared pointer to an object to track for
      *   these callbacks. If set, a weak pointer will be created to this
      *   object, and if the reference count goes to zero the subscriber
      *   callbacks will not get called. Note that setting this will cause
      *   a new reference to be added to the object before the callback, and
      *   for it to go out of scope (and potentially be deleted) in the code
      *   path (and therefore thread) that the callback is invoked from. 
      * \return On success, a ROS service server that, when all copies of it
      *   go out of scope, will unadvertise this service.
      * 
      * \see advertiseService(const std::string&, const ros::AdvertiseServiceOptions&)
      *   for a detailed description of the method's behavior.
      */
    template <typename Callback> ros::ServiceServer advertiseService(const
      std::string& param, const std::string& defaultService, Callback callback,
      const ros::VoidConstPtr& trackedObject = ros::VoidConstPtr());
    
    /** \brief Advertise a service, with full range of options
      * 
      * Extending the standard ROS interface for advertising services, this
      * implementation takes a parameter name under which the service server
      * configuration may be retrieved from the parameter server. In
      * particular, this parameter is expected to represent a structure of
      * the following YAML format:
      * 
      * - servers:
      *   - param:
      *     - service: <string>
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new service server.
      * \param[in] defaultOptions The default advertise options to use.
      * \return On success, a ROS service server that, when all copies of it
      *   go out of scope, will unadvertise this service.
      * 
      * \see ros::NodeHandle::advertiseService
      */
    ros::ServiceServer advertiseService(const std::string& param,
      const ros::AdvertiseServiceOptions& defaultOptions);
    
    /** \brief Create a client for a service, with standard options
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new service client.
      * \param[in] defaultService The default name of the service to
      *   connect to.
      * \param[in] defaultPersistent  Whether this connection should persist
      *   by default. Persistent services keep the connection to the remote
      *   host active so that subsequent calls will happen faster. In general
      *   persistent services are discouraged, as they are not as robust to
      *   node failure as non-persistent services. In fact, to maintain
      *   persistent service connections, the client needs to explicitly
      *   implement the re-connection strategies for recovering from potential
      *   failures when calling the service server.
      * \param[in] headerValues Key/value pairs you would like to send along
      *   in the connection handshake.
      * \return On success, a ROS service client that, when all copies of it
      *   go out of scope, will disconnect from this service.
      * 
      * \see serviceClient(const std::string&, const ros::ServiceClientOptions&)
      *   for a detailed description of the method's behavior.
      */
    template <class S> ros::ServiceClient serviceClient(const std::string&
      param, const std::string& defaultService, bool defaultPersistent =
      false, const ros::M_string& headerValues = ros::M_string());
    
    /** \brief Create a client for a service, with full range of options
      * 
      * Extending the standard ROS interface for creating service clients,
      * this implementation takes a parameter name under which the service
      * client configuration may be retrieved from the parameter server. In
      * particular, this parameter is expected to represent a structure of
      * the following YAML format:
      * 
      * - clients:
      *   - param:
      *     - service: <string>
      *     - persistent: <bool>
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new service client.
      * \param[in] defaultOptions The default service client options to use.
      * \return On success, a ROS service client that, when all copies of it
      *   go out of scope, will disconnect from this service.
      * 
      * \see ros::NodeHandle::serviceClient
      */
    ros::ServiceClient serviceClient(const std::string& param, const
      ros::ServiceClientOptions& defaultOptions);
  };
};

#include <roscpp_nodewrap/NodeImpl.tpp>

#include <roscpp_nodewrap/Console.h>

#endif
