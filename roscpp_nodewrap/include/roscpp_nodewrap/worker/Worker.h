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

/** \file Worker.h
  * \brief Header file providing the Worker class interface
  */

#ifndef ROSCPP_NODEWRAP_WORKER_H
#define ROSCPP_NODEWRAP_WORKER_H

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <roscpp_nodewrap/Forwards.h>

#include <roscpp_nodewrap/worker/WorkerEvent.h>
#include <roscpp_nodewrap/worker/WorkerOptions.h>

#include <roscpp_nodewrap_msgs/GetWorkerState.h>

namespace nodewrap {
  /** \brief Abstract ROS node worker
    * 
    * This class provides the abstract basis of a worker for use with
    * the ROS node implementation.
    */
  class Worker {
  friend class WorkerManager;
  friend class WorkerQueueCallback;
  public:
    /** \brief Default constructor
      */
    Worker();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source worker which is being copied to this worker.
      */
    Worker(const Worker& src);
    
    /** \brief Destructor
      */
    ~Worker();
    
    /** \brief Access this worker's name
      */
    std::string getName() const;
    
    /** \brief Start the worker
      */
    void start();
      
    /** \brief Cancel the worker
      */
    void cancel(bool block = false);
            
    /** \brief Wake the worker
      */
    void wake();
    
    /** \brief Perform shutdown of the worker
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const Worker& worker) const {
      return (impl < worker.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const Worker& worker) const {
      return (impl == worker.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const Worker& worker) const {
      return (impl != worker.impl);
    };
    
  protected:
    /** \brief ROS node worker implementation
      * 
      * This class provides the private implementation of the node worker.
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const WorkerOptions& defaultOptions,
        const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the ROS node handle of the node implementation
        *   owning this worker
        */
      ros::NodeHandle& getNodeHandle() const;
      
      /** \brief Query if this worker is valid
        */
      bool isValid() const;
      
      /** \brief Start the worker (implementation)
        */
      void start();
            
      /** \brief Start the worker (thread-safe, abstract declaration)
        */
      virtual void safeStart() = 0;
            
      /** \brief Wake the worker (implementation)
        */
      void wake();
            
      /** \brief Wake the worker (thread-safe, abstract declaration)
        */
      virtual void safeWake() = 0;
            
      /** \brief Cancel the worker (implementation)
        */
      void cancel(bool block = false);
            
      /** \brief Stop the worker (thread-safe, abstract declaration)
        */
      virtual void safeStop() = 0;
            
      /** \brief Unadvertise the worker's services
        */
      void unadvertise();
            
      /** \brief Run this worker once
        */ 
      void runOnce();
      
      /** \brief Service callback starting this worker
        */ 
      bool startCallback(std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response);
      
      /** \brief Service callback canceling this worker
        */ 
      bool cancelCallback(std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response);
      
      /** \brief Service callback for querying the state of this worker
        */ 
      bool getStateCallback(GetWorkerState::Request& request,
        GetWorkerState::Response& response);
      
      /** \brief The name of this worker
        */ 
      std::string name;
      
      /** \brief The expected cycle time of this worker
        */ 
      ros::Duration expectedCycleTime;
      
      /** \brief True, if the worker will be started automatically
        */ 
      bool autostart;
      
      /** \brief The worker's callback
        */ 
      WorkerCallback callback;
      
      /** \brief True, if the worker has been started
        */ 
      bool started;
      
      /** \brief True, if the worker has been canceled
        */ 
      bool canceled;
      
      /** \brief Start time of the worker
        */ 
      ros::Time startTime;

      /** \brief Last cycle time of the worker
        */ 
      ros::Time lastCycleTime;

      /** \brief The actual cycle time of this worker
        */ 
      ros::Duration actualCycleTime;
      
      /** \brief Identifier of the thread running the worker
        * 
        * \note The identifier is valid only if the worker is currently
        *   active.
        */ 
      boost::thread::id threadId;
      
      /** \brief Service server starting this worker
        */ 
      ros::ServiceServer startServer;
      
      /** \brief Service server canceling this worker
        */ 
      ros::ServiceServer cancelServer;
      
      /** \brief Service server for querying the state of this worker
        */ 
      ros::ServiceServer getStateServer;
      
      /** \brief The worker's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The worker's cancellation condition
        */ 
      boost::condition cancelCondition;
      
      /** \brief The node implementation owning this worker
        */ 
      NodeImplPtr nodeImpl;
    };
    
    /** \brief Declaration of the worker implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the worker implementation weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The worker's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    Worker(const ImplPtr& impl);
  };
};

#endif
