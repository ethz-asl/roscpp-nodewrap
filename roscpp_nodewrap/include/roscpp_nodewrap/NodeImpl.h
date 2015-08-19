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

#ifndef ROSCPP_NODEWRAP_NODE_IMPL_H
#define ROSCPP_NODEWRAP_NODE_IMPL_H

#include <map>

#include <boost/enable_shared_from_this.hpp>

#include <ros/console.h>

#include <roscpp_nodewrap/Publisher.h>
#include <roscpp_nodewrap/PublisherOptions.h>
#include <roscpp_nodewrap/Subscriber.h>
#include <roscpp_nodewrap/SubscriberOptions.h>
#include <roscpp_nodewrap/ServiceServer.h>
#include <roscpp_nodewrap/ServiceServerOptions.h>
#include <roscpp_nodewrap/ServiceClient.h>
#include <roscpp_nodewrap/ServiceClientOptions.h>

#include <roscpp_nodewrap/timer/TimerManager.h>

#include <roscpp_nodewrap/diagnostics/DiagnosticUpdater.h>
#include <roscpp_nodewrap/diagnostics/FunctionTask.h>

#include <roscpp_nodewrap/worker/Worker.h>
#include <roscpp_nodewrap/worker/WorkerManager.h>
#include <roscpp_nodewrap/worker/WorkerOptions.h>

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
    public virtual NodeInterface,
    public boost::enable_shared_from_this<NodeImpl> {
  template <class C> friend class Node;
  template <class C> friend class Nodelet;
  friend class Publisher;
  friend class Subscriber;
  friend class ServiceServer;
  friend class ServiceClient;
  friend class TimerManager;
  friend class Timer;
  friend class DiagnosticUpdater;
  friend class DiagnosticTask;
  friend class Worker;
  friend class WorkerManager;
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
    /** \brief Set the hardware identifier of the node's diagnostic updater
      * 
      * \param[in] hardwareId The hardware identifier to be announced by
      *   the node's diagnostic updater.
      */
    void setDiagnosticsHardwareId(const std::string& hardwareId);
      
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
    
    /** \brief Node(let) signal handler
      * 
      * This helper method will be called if the process running the node(let)
      * receives a SIGINT. The default behavior then is to simply initiate its
      * shutdown. However, the method is declared virtual such that this
      * behavior can be adapted in the implementation of the derived node(let).
      * 
      * \param[in] signal The signal to be handled which, by implementation,
      *   should always equate to SIGINT.
      * 
      * \see shutdown
      */
    virtual void signaled(int signal);
    
    /** \brief Unload the nodelet
      *
      * This helper method will be called from the nodelet template wrapper
      * of the node(let) implementation if the nodelet is unloaded. The default
      * behavior then is to simply initiate its shutdown. However, the method
      * is declared virtual such that this behavior can be adapted in the
      * implementation of the derived node(let).
      * 
      * \see shutdown
      */
    virtual void unload();

    /** \brief Create a high precision timer, with standard options
      * 
      * \param[in] period The period of the timer.
      * \param[in] callback A member function pointer to call when the
      *   timer event occurs.
      * \param[in] oneshot If true, the timer will be non-cyclic.
      * \param[in] autostart If true, the timer will be started
      *   immediately.
      * \return On success, a high precision timer that, when all copies of
      *   it go out of scope, will be stopped.
      * 
      * \see createTimer(const ros::TimerOptions&) for a detailed description
      *   of the method's behavior.
      */
    template <class T> Timer createTimer(const ros::Duration& period,
      void(T::*callback)(const ros::TimerEvent&), bool oneshot = false,
      bool autostart = true);
    
    /** \brief Create a high precision timer, with full range of options
      * 
      * This method creates a high precision timer for use with the ROS
      * node implementation. Its interface is identical to that of the
      * ROS standard timer, but its implementation features microsecond
      * precision.
      * 
      * \param[in] options The timer options to use.
      * \return On success, a high precision timer that, when all copies of
      *   it go out of scope, will be stopped.
      */
    Timer createTimer(const ros::TimerOptions& options);
    
    /** \brief Advertise a topic, with standard options
      * 
      * \param[in] name The name of the new publisher, a valid ROS graph
      *   resource name.
      * \param[in] defaultTopic The default topic to advertise on.
      * \param[in] defaultQueueSize The default maximum number of outgoing
      *   messages to be queued for delivery to subscribers.
      * \param[in] defaultLatch If true, the last message published on
      *   this topic will by default be saved and sent to new subscribers
      *   when they connect.
      * \return On success, a ROS publisher that, when it goes out of scope,
      *   will automatically release a reference on this advertisement. On
      *   failure, an empty ROS publisher.
      * 
      * \see advertise(const std::string&, const PublisherOptions&) for
      *   a detailed description of the method's behavior.
      */
    template <class M> Publisher advertise(const std::string& name,
      const std::string& defaultTopic, uint32_t defaultQueueSize, bool
      defaultLatch = false);
    
    /** \brief Advertise a topic, with most of the avaiable options
      * 
      * \param[in] name The name of the new publisher, a valid ROS graph
      *   resource name.
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
      * \return On success, a ROS publisher that, when it goes out of scope,
      *   will automatically release a reference on this advertisement. On
      *   failure, an empty ROS publisher.
      * 
      * \see advertise(const std::string&, const PublisherOptions&) for
      *   a detailed description of the method's behavior.
      */
    template <class M> Publisher advertise(const std::string& name,
      const std::string& defaultTopic, uint32_t defaultQueueSize, const
      ros::SubscriberStatusCallback& connectCallback, const
      ros::SubscriberStatusCallback& disconnectCallback = 
      ros::SubscriberStatusCallback(), const ros::VoidConstPtr&
      trackedObject = ros::VoidConstPtr(), bool defaultLatch = false);
    
    /** \brief Advertise a topic, with full range of options
      * 
      * Extending the standard ROS interface for advertising topics, this
      * implementation takes a name under which the publisher configuration
      * may be retrieved from the parameter server. In particular, this
      * configuration is expected to represent a structure of the following
      * YAML format:
      * 
      * - publishers:
      *   - name:
      *     - topic: <string>
      *     - queue_size: <int>
      *     - latch: <bool>
      * 
      * \param[in] name The name of the new publisher, a valid ROS graph
      *   resource name.
      * \param[in] defaultOptions The default publisher options to use.
      * \return On success, a ROS publisher that, when it goes out of scope,
      *   will automatically release a reference on this advertisement. On
      *   failure, an empty ROS publisher.
      * 
      * \see ros::NodeHandle::advertise
      */
    Publisher advertise(const std::string& name, const PublisherOptions&
      defaultOptions);
    
    /** \brief Subscribe to a topic, with standard options (non-const version)
      * 
      * \param[in] name The name of the new subscriber, a valid ROS graph
      *   resource name.
      * \param[in] defaultTopic The default topic to subscribe to.
      * \param[in] defaultQueueSize The default number of incoming messages
      *   to queue up for processing (messages in excess of this queue
      *   capacity will be discarded). 
      * \param[in] callback A member function pointer to call when a message
      *   has arrived.
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
    template <class M, class T> Subscriber subscribe(const std::string& 
      name, const std::string& defaultTopic, uint32_t defaultQueueSize,
      void(T::*callback)(const boost::shared_ptr<M const>&), const
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
    template <class M, class T> Subscriber subscribe(const std::string& 
      name, const std::string& defaultTopic, uint32_t defaultQueueSize,
      void(T::*callback)(const boost::shared_ptr<M const>&) const,
      const ros::TransportHints& transportHints = ros::TransportHints());
    
    /** \brief Subscribe to a topic, with full range of options
      * 
      * Extending the standard ROS interface for subscribing to topics, this
      * implementation takes a name under which the subscribe configuration
      * may be retrieved from the parameter server. In particular, this
      * configuration is expected to represent a structure of the following
      * YAML format:
      * 
      * - subscribers:
      *   - name:
      *     - topic: <string>
      *     - queue_size: <int>
      * 
      * \param[in] name The name of the new subscriber, a valid ROS graph
      *   resource name.
      * \param[in] defaultOptions The default subscriber options to use.
      * \return On success, a ROS subscriber that, when all copies of it go
      *   out of scope, will unsubscribe from this topic.
      * 
      * \see ros::NodeHandle::subscribe
      */
    Subscriber subscribe(const std::string& name, const SubscriberOptions&
      defaultOptions);
    
    /** \brief Advertise a service, templated on two message types and with
      *   standard options
      * 
      * \param[in] name The name of the new service server, a valid ROS graph
      *   resource name.
      * \param[in] defaultService The default service name to advertise on.
      * \param[in] callback A member function pointer to call when the service
      *   is called.
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
    template <class MReq, class MRes, class T> ServiceServer
      advertiseService(const std::string& name, const std::string&
      defaultService, bool(T::*callback)(MReq&, MRes&), const
      ros::VoidConstPtr& trackedObject = ros::VoidConstPtr());
    
    /** \brief Advertise a service, templated on the service type and with
      *   standard options
      * 
      * \param[in] name The name of the new service server, a valid ROS graph
      *   resource name.
      * \param[in] defaultService The default service name to advertise on.
      * \param[in] callback A member function pointer to call when the service
      *   is called.
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
    template <class S, class T> ServiceServer advertiseService(
      const std::string& name, const std::string& defaultService,
      bool(T::*callback)(typename S::Request&, typename S::Response&),
      const ros::VoidConstPtr& trackedObject = ros::VoidConstPtr());
    
    /** \brief Advertise a service, with full range of options
      * 
      * Extending the standard ROS interface for advertising services, this
      * implementation takes a name under which the service server
      * configuration may be retrieved from the parameter server. In
      * particular, this configuration is expected to represent a structure
      * of the following YAML format:
      * 
      * - servers:
      *   - name:
      *     - service: <string>
      * 
      * \param[in] name The name of the new service server, a valid ROS graph
      *   resource name.
      * \param[in] defaultOptions The default advertise options to use.
      * \return On success, a ROS service server that, when all copies of it
      *   go out of scope, will unadvertise this service.
      * 
      * \see ros::NodeHandle::advertiseService
      */
    ServiceServer advertiseService(const std::string& name, const
      ServiceServerOptions& defaultOptions);
    
    /** \brief Create a client for a service, templated on two message types
      *   and with standard options
      * 
      * \param[in] name The name of the new service client, a valid ROS graph
      *   resource name.
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
    template <class MReq, class MRes> ServiceClient serviceClient(
      const std::string& name, const std::string& defaultService, bool
      defaultPersistent = false, const ros::M_string& headerValues =
      ros::M_string());
    
    /** \brief Create a client for a service, templated on the service type
      *   and with standard options
      * 
      * \param[in] name The name of the new service client, a valid ROS graph
      *   resource name.
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
    template <class S> ServiceClient serviceClient(const std::string&
      name, const std::string& defaultService, bool defaultPersistent =
      false, const ros::M_string& headerValues = ros::M_string());
    
    /** \brief Create a client for a service, with full range of options
      * 
      * Extending the standard ROS interface for creating service clients,
      * this implementation takes a name under which the service client
      * configuration may be retrieved from the parameter server. In
      * particular, this configuration is expected to represent a structure
      * of the following YAML format:
      * 
      * - clients:
      *   - name:
      *     - service: <string>
      *     - persistent: <bool>
      * 
      * \param[in] name The name of the parameter which stores the
      *   configuration of the new service client.
      * \param[in] defaultOptions The default service client options to use.
      * \return On success, a ROS service client that, when all copies of it
      *   go out of scope, will disconnect from this service.
      * 
      * \see ros::NodeHandle::serviceClient
      */
    ServiceClient serviceClient(const std::string& name, const
      ServiceClientOptions& defaultOptions);    

    /** \brief Add a worker, with standard options
      * 
      * \param[in] name The name of the new worker, a valid ROS graph
      *   resource name.
      * \param[in] defaultFrequency The default frequency at which the
      *   worker's work callback will be attempted to be invoked by the
      *   worker's timer.
      * \param[in] callback A member function pointer to call when the worker
      *   should perform its work.
      * \param[in] defaultAutostart If true, the worker will by default be
      *   started automatically.
      * \param[in] synchronous If true, the worker will be synchronous.
      * \return On success, a worker that, when all copies of it go out
      *   of scope, will remove this worker.
      */
    template <class T> Worker addWorker(const std::string& name, const
      double defaultFrequency, bool(T::*callback)(const WorkerEvent&),
      bool defaultAutostart = true, bool synchronous = false);
    
    /** \brief Add a worker, with full range of options
      * 
      * \param[in] name The name of the new worker, a valid ROS graph
      *   resource name.
      * \param[in] defaultOptions The default worker options to use.
      * \return On success, a worker that, when all copies of it go out
      *   of scope, will remove this worker.
      */
    Worker addWorker(const std::string& name, const WorkerOptions&
      defaultOptions);
    
    /** \brief Add a diagnostic task, with full range of options
      * 
      * \param[in] name The name of the new task.
      * \param[in] defaultOptions The default diagnostic task options to use.
      * \return On success, a diagnostic task that, when all copies of it go
      *   out of scope, will remove this task.
      */
    template <class T> T addDiagnosticTask(const std::string& name,
      const typename T::Options& defaultOptions = typename T::Options());
    
    /** \brief Add a diagnostic function task
      * 
      * \param[in] name The name of the new function task.
      * \param[in] callback A member function pointer to call when the
      *   task should perform diagnostics.
      * \return On success, a diagnostic function task that, when all copies
      *   of it go out of scope, will remove this task.
      */
    template <class T> FunctionTask addDiagnosticTask(const std::string&
      name, void(T::*callback)(diagnostic_updater::DiagnosticStatusWrapper&)
      const);
    
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
    
    /** \brief The node implementation's private timer manager
      */
    TimerManager timerManager;
    
    /** \brief The node implementation's private diagnostic updater
      */
    DiagnosticUpdater diagnosticUpdater;
    
    /** \brief The node implementation's worker manager
      */
    WorkerManager workerManager;

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
    
public:
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
