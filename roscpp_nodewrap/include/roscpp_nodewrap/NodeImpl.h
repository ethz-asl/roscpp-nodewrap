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

#include <map>

#include <boost/enable_shared_from_this.hpp>

#include <ros/console.h>

#include <roscpp_nodewrap/NodeInterface.h>

#include <roscpp_nodewrap/ConfigServer.h>
#include <roscpp_nodewrap/ParamClient.h>
#include <roscpp_nodewrap/ParamClientOptions.h>
#include <roscpp_nodewrap/ParamServer.h>
#include <roscpp_nodewrap/ParamServerOptions.h>

namespace nodewrap {
  /** \brief Abstract class implementation of a ROS node(let)
    * 
    * This class provides the abstract, opaque implementation of a ROS
    * node(let). It should be inherited and implemented by all node wrappers
    * which are intended to become instantiable as both, ROS nodes and ROS
    * nodelets.
    */
  class NodeImpl :
    public virtual NodeInterface,
    public boost::enable_shared_from_this<NodeImpl> {
  friend class ConfigClient;
  friend class ConfigServer;
  template <class C> friend class Node;
  template <class C> friend class Nodelet;
  friend class ParamClient;
  friend class ParamServer;
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
    
    /** \brief Retrieve the node(let)'s name
      * 
      * \return The node(let)'s name.
      * 
      * \see NodeInterface::getName
      */
    const std::string& getName() const;
    
    /** \brief Query if the node(let) is a ROS nodelet
      * 
      * \return True, if the node(let) implements a ROS nodelet.
      * 
      * \see NodeInterface::isNodelet
      */
    bool isNodelet() const;
    
    /** \brief Query if the node(let) is valid
      * 
      * \return True, if the node(let) has a valid ROS node handle.
      * 
      * \see ros::NodeHandle::ok
      */
    bool isValid() const;
    
  protected:
    /** \brief Retrieve the node(let)'s ROS node handle
      * 
      * \return The ROS node handle used by this node(let).
      * 
      * \see NodeInterface::getNodeHandle
      */
    ros::NodeHandle& getNodeHandle() const;
    
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
    
    /** \brief Retrieve advertise options from the parameter server
      * 
      * \param[in] key The key referring to the publisher whose advertise
      *   options shall be retrieved.
      * \param[in] defaultOptions The publisher's default advertise options.
      * \return The actual advertise options if the corresponding parameters
      *   have been defined or their default values otherwise.
      * 
      * \see getParam
      */
    ros::AdvertiseOptions getAdvertiseOptions(const std::string& key, const
      ros::AdvertiseOptions& defaultOptions) const;

    /** \brief Retrieve subscribe options from the parameter server
      * 
      * \param[in] key The key referring to the subscriber whose subscribe
      *   options shall be retrieved.
      * \param[in] defaultOptions The subscriber's default subscribe options.
      * \return The actual subscribe options if the corresponding parameters
      *   have been defined or their default values otherwise.
      * 
      * \see getParam
      */
    ros::SubscribeOptions getSubscribeOptions(const std::string& key, const
      ros::SubscribeOptions& defaultOptions) const;

    /** \brief Retrieve advertise service options from the parameter server
      * 
      * \param[in] key The key referring to the service server whose advertise
      *   service options shall be retrieved.
      * \param[in] defaultOptions The service server's default advertise
      *   service options.
      * \return The actual advertise service options if the corresponding
      *   parameters have been defined or their default values otherwise.
      * 
      * \see getParam
      */
    ros::AdvertiseServiceOptions getAdvertiseServiceOptions(const std::string&
      key, const ros::AdvertiseServiceOptions& defaultOptions) const;

    /** \brief Retrieve service client options from the parameter server
      * 
      * \param[in] key The key referring to the service client whose service
      *   client options shall be retrieved.
      * \param[in] defaultOptions The service client's default service client
      *   options.
      * \return The actual service client options if the corresponding
      *   parameters have been defined or their default values otherwise.
      * 
      * \see getParam
      */
    ros::ServiceClientOptions getServiceClientOptions(const std::string& key,
      const ros::ServiceClientOptions& defaultOptions) const;

    /** \brief Perform node(let) initialization
      * 
      * Here, all initialization code should be placed in the implementation,
      * including topic advertisement and subscription, service connection,
      * and parameter retrieval.
      * 
      * \note This function must be implemented by the subclass.
      */
    virtual void init() = 0;

    /** \brief Perform node(let) cleanup
      * 
      * Here, any cleanup code to be executed on node(let) shutdown should be
      * placed in the implementation.
      * 
      * \note This function must be implemented by the subclass.
      */
    virtual void cleanup() = 0;
    
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
      *   will automatically release a reference on this advertisement. On
      *   failure, an empty ROS publisher.
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
    
    /** \brief Advertise a service, templated on two message types and with
      *   standard options
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new service server.
      * \param[in] defaultService The default service name to advertise on.
      * \param[in] fp A member function pointer to call when the service is
      *   called.
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
    template <class MReq, class MRes, class T> ros::ServiceServer
      advertiseService(const std::string& param, const std::string&
      defaultService, bool(T::*fp)(MReq&, MRes&), const ros::VoidConstPtr&
      trackedObject = ros::VoidConstPtr());
    
    /** \brief Advertise a service, templated on the service type and with
      *   standard options
      * 
      * \param[in] param The name of the parameter which stores the
      *   configuration of the new service server.
      * \param[in] defaultService The default service name to advertise on.
      * \param[in] fp A member function pointer to call when the service is
      *   called.
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
    template <class S, class T> ros::ServiceServer advertiseService(const
      std::string& param, const std::string& defaultService, bool(T::*fp)(S&),
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
    
    /** \brief Create a client for a service, templated on two message types
      *   and with standard options
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
    template <class MReq, class MRes> ros::ServiceClient serviceClient(
      const std::string& param, const std::string& defaultService, bool
      defaultPersistent = false, const ros::M_string& headerValues =
      ros::M_string());
    
    /** \brief Create a client for a service, templated on the service type
      *   and with standard options
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
    
    /** \brief Advertise a parameter service, templated on the parameter type
      *   and with standard options
      * 
      * \param[in] key The key of the parameter to be advertised.
      * \param[in] service The parameter service name to advertise on. If
      *   empty, the provided parameter key will instead be used to construct
      *   the service name as params/key.
      * \param[in] name The name of the parameter under which it can be
      *   accessed through rosparam. If empty, the provided parameter key
      *   will instead be assumed to also correspond to the parameter name,
      *   in which case it would be interpreted as a relative graph resource
      *   name.
      * \param[in] cached If true, the parameter service server will provide
      *   the cached parameter value.
      * \return On success, a parameter service server that, when all copies
      *   of it go out of scope, will unadvertise this parameter service.
      * 
      * \see advertiseParam(const std::string&, const ParamServerOptions&)
      *   for a detailed description of the method's behavior.
      */
    template <typename P> ParamServer advertiseParam(const std::string& key,
      const std::string& service = std::string(), const std::string& name =
      std::string(), bool cached = true);
    
    /** \brief Advertise a parameter service, templated on the parameter
      *   specification and with standard options
      * 
      * \param[in] key The key of the parameter to be advertised.
      * \param[in] fromXmlRpcValue Function for assigning the parameter's
      *   value from an XML/RPC value.
      * \param[in] toXmlRpcValue Function for assigning the parameter's
      *   value to an XML/RPC value.
      * \param[in] fromRequest Function for assigning the parameter's
      *   value from a service request.
      * \param[in] toResponse Function for assigning the parameter's
      *   value to a service response.
      * \param[in] service The parameter service name to advertise on. If
      *   empty, the provided parameter key will instead be used to construct
      *   the service name as params/key.
      * \param[in] name The name of the parameter under which it can be
      *   accessed through rosparam. If empty, the provided parameter key
      *   will instead be assumed to also correspond to the parameter name,
      *   in which case it would be interpreted as a relative graph resource
      *   name.
      * \param[in] cached If true, the parameter service server will provide
      *   the cached parameter value.
      * \return On success, a parameter service server that, when all copies
      *   of it go out of scope, will unadvertise this parameter service.
      * 
      * \see advertiseParam(const std::string&, const ParamServerOptions&)
      *   for a detailed description of the method's behavior.
      */
    template <class PSpec> ParamServer advertiseParam(const std::string&
      key, const typename PSpec::FromXmlRpcValue& fromXmlRpcValue, const
      typename PSpec::ToXmlRpcValue& toXmlRpcValue, const typename
      PSpec::FromRequest& fromRequest, const typename PSpec::ToResponse&
      toResponse, const std::string& service = std::string(), const
      std::string& name = std::string(), bool cached = true);
    
    /** \brief Advertise a parameter service, with full range of options
      * 
      * To advertise a parameter service, this method employs the service
      * name specified in the parameter server options as parent namespace
      * for the services. The service signatures further depend on the
      * value type of the advertised parameter and must be specified as
      * template parameters in the templated initializer of the parameter
      * server options. Similarly, the types of the functions required to
      * assign the parameter value from/to its XML/RPC value representation
      * and the service request/response types depend on the value type, and
      * their function object wrappers must be specified when initializing
      * the parameter server options.
      * 
      * Once successfully created, the parameter service server will advertise
      * the following services:
      * 
      * - service/get_value: <GetParamValue>
      * - service/set_value: <SetParamValue>
      * - service/get_info: <GetParamInfo>
      * 
      * where <GetParamValue> and <SetParamValue> would correspond to the
      * service types defined for the advertised parameter.
      * 
      * On creation of the first parameter service server, a configuration
      * service server will be instantiated simultaneously. It provides
      * information about all parameter services which have been advertised
      * for this node. In particular, the configuration services comprise:
      * 
      * - list_params: <ListParams>
      * - has_params: <HasParam>
      * 
      * and may be called to identify a node's parameter services.
      * 
      * \param[in] key The key of the parameter to be advertised.
      * \param[in] options The parameter service server options to use.
      * \return On success, a parameter service server that, when all copies
      *   of it go out of scope, will unadvertise this parameter service.
      */
    ParamServer advertiseParam(const std::string& key, const
      ParamServerOptions& options);

    /** \brief Create a client for a parameter service, templated on the
      *   parameter type and with standard options
      * 
      * \param[in] service The name of the parameter service the parameter
      *   service client connects to.
      * \param[in] persistent Whether the connection of the parameter service
      *   client should persist. 
      * \return On success, a parameter service client that, when all copies
      *   of it go out of scope, will disconnect from the parameter service.
      * 
      * \see paramClient(const ParamClientOptions&) for a detailed description
      *   of the method's behavior.
      */
    template <typename P> ParamClient paramClient(const std::string& service,
      bool persistent = false);
    
    /** \brief Create a client for a parameter service, templated on the
      *   parameter specification and with standard options
      * 
      * \param[in] service The name of the parameter service the parameter
      *   service client connects to.
      * \param[in] fromResponse Function for assigning the parameter's
      *   value from a service response.
      * \param[in] toRequest Function for assigning the parameter's
      *   value to a service request.
      * \param[in] persistent Whether the connection of the parameter service
      *   client should persist. 
      * \return On success, a parameter service client that, when all copies
      *   of it go out of scope, will disconnect from the parameter service.
      * 
      * \see paramClient(const ParamClientOptions&) for a detailed description
      *   of the method's behavior.
      */
    template <class PSpec> ParamClient paramClient(const std::string&
      service, const typename PSpec::FromResponse& fromResponse, const
      typename PSpec::ToRequest& toRequest, bool persistent = false);
    
    /** \brief Create a client for a parameter service, with full range of
      *   options
      * 
      * To create a client for a parameter service, this method interprets
      * the service name specified in the parameter client options as
      * parent namespace for the services. The service signatures further
      * depend on the value type of the advertised parameter and must be
      * specified as template parameters in the templated initializer of the
      * parameter client options. Similarly, the types of the functions
      * required to assign the parameter value from/to its service
      * request/response types depend on the value type, and their function
      * object wrappers must be specified when initializing the parameter
      * client options.
      * 
      * \param[in] options The parameter service client options to use.
      * \return On success, a parameter service client that, when all copies
      *   of it go out of scope, will disconnect from the parameter service.
      */
    ParamClient paramClient(const ParamClientOptions& options);

  private:
    /** \brief The node implementation's name
      */
    std::string name;
    
    /** \brief True, if the node(let) implements a ROS nodelet
      */
    bool nodelet;
    
    /** \brief The node implementation's private ROS node handle
      */
    ros::NodeHandlePtr nodeHandle;
    
    /** \brief The node implementation's configuration service server
      */
    ConfigServer configServer;
    
    /** \brief The node implementation's parameter service servers
      */
    std::map<std::string, ParamServer> paramServers;
    
    /** \brief Start the node(let)
      *
      * This helper method is called from the template wrapper of
      * the node(let) to initialize its members. Once all members have
      * been initialized, the method invokes init to perform the
      * implementation-specific initialization of the node(let).
      * 
      * \see init
      */
    void start(const std::string& name, bool nodelet, const
      ros::NodeHandlePtr& nodeHandle);
    
    /** \brief Shutdown the node(let)
      *
      * This method is executed if a process interrupt signals the ROS
      * node(let) to shutdown.
      * 
      * Note that the method is called from a custom signal handler, just
      * before shutting down the ROS node of the underlying process. It
      * will, for instance, not be invoked in the case of a programmatic
      * shutdown (or unloading) of the ROS node(let).
      * 
      * \see ros::shutdown and cleanup
      */
    void shutdown();
  };
};

#include <roscpp_nodewrap/NodeImpl.tpp>

#include <roscpp_nodewrap/Console.h>

#endif
