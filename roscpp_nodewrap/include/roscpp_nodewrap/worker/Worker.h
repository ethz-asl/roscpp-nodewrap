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

#include <roscpp_nodewrap/Managed.h>

#include <roscpp_nodewrap/worker/WorkerEvent.h>
#include <roscpp_nodewrap/worker/WorkerImpl.h>
#include <roscpp_nodewrap/worker/WorkerOptions.h>

#include <roscpp_nodewrap/diagnostics/StatefulFrequencyTask.h>
#include <roscpp_nodewrap/diagnostics/WorkerStatusTask.h>

#include <roscpp_nodewrap_msgs/GetWorkerFrequency.h>
#include <roscpp_nodewrap_msgs/GetWorkerState.h>

namespace nodewrap {
  /** \brief Abstract basis of the ROS node worker
    * 
    * This class provides the abstract basis of a worker for use with
    * the ROS node implementation.
    */
  class Worker :
    public Managed<Worker, std::string> {
  friend class WorkerManager;
  friend class WorkerQueueCallback;
  friend class WorkerStatusTask;
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
    
    /** \brief Retrieve this worker's name
      */
    std::string getName() const;
    
    /** \brief Retrieve the time passed since the worker has been started
      */
    ros::Duration getTimeSinceStart() const;
      
    /** \brief Retrieve the frequency statistics' estimates of this worker
      * 
      * \return The frequency statistics' estimates of this worker.
      */
    FrequencyStatistics::Estimates getStatisticsEstimates() const;
      
    /** \brief True, if this worker has been canceled
      */
    bool isCanceled() const;
    
    /** \brief Start the worker
      */
    void start();
      
    /** \brief Cancel the worker
      */
    void cancel(bool block = false);
            
    /** \brief Wake the worker
      */
    void wake();
    
  protected:
    /** \brief Abstract basis of the ROS node worker implementation
      * 
      * This class provides the protected, abstract basis of the node
      * worker implementation.
      */
    class Impl :
      public Managed<Worker, std::string>::Impl,
      public WorkerImpl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the frequency statistics' estimates of this worker
        */
      FrequencyStatistics::Estimates getStatisticsEstimates() const;
      
      /** \brief Retrieve the time passed since the worker has been started
        *   (implementation)
        */
      ros::Duration getTimeSinceStart() const;
        
      /** \brief True, if this worker has been canceled (implementation)
        */
      bool isCanceled() const;
    
      /** \brief True, if this worker implementation is valid
        */
      bool isValid() const;      
      
      /** \brief Initialize the worker
        */
      void init(const WorkerOptions& defaultOptions);
      
      /** \brief Create a timer for this worker
        */
      Timer createTimer(const ros::TimerOptions& options);
      
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
      void shutdown();
            
      /** \brief Run this worker once
        */ 
      void runOnce();
      
      /** \brief Spin over the private callback queue of this worker
        */ 
      void spin();
      
      /** \brief Service callback starting this worker
        */ 
      bool startCallback(std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response);
      
      /** \brief Service callback canceling this worker
        */ 
      bool cancelCallback(std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response);
      
      /** \brief Service callback for querying the frequency of this worker
        */ 
      bool getFrequencyCallback(GetWorkerFrequency::Request& request,
        GetWorkerFrequency::Response& response);
      
      /** \brief Service callback for querying the state of this worker
        */ 
      bool getStateCallback(GetWorkerState::Request& request,
        GetWorkerState::Response& response);
      
      /** \brief The expected cycle time of this worker
        */ 
      ros::Duration expectedCycleTime;
      
      /** \brief True, if the worker will be started automatically
        */ 
      bool autostart;
      
      /** \brief The worker's callback
        */ 
      WorkerCallback callback;
      
      /** \brief The callback queue used by this worker
        */ 
      ros::CallbackQueueInterface* callbackQueue;
    
      /** \brief If true, this worker's callback queue is private
        */ 
      bool hasPrivateCallbackQueue;
      
      /** \brief The expected priority of the spinner serving this worker's 
        *   private callback queue
        */ 
      int expectedPriority;
      
      /** \brief The actual priority of the spinner serving this worker's 
        *   private callback queue
        */ 
      int actualPriority;
      
      /** \brief A shared pointer to an object being tracked for the worker
        *   callbacks
        */ 
      ros::VoidConstWPtr trackedObject;
      
      /** \brief If true, the worker has an object to track for its
        *   callbacks
        */ 
      bool hasTrackedObject;
      
      /** \brief True, if the worker has been started
        */ 
      bool started;
      
      /** \brief True, if the worker has been canceled
        */ 
      bool canceled;
      
      /** \brief Start time of the worker
        */ 
      ros::Time startTime;

      /** \brief Time of the last worker cycle
        */ 
      ros::Time timeOfLastCycle;

      /** \brief The actual cycle time of this worker
        */ 
      ros::Duration actualCycleTime;
      
      /** \brief The spinner serving this worker's private callback queue
        */ 
      boost::thread spinner;
      
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
      
      /** \brief Service server for querying the frequency of this worker
        */ 
      ros::ServiceServer getFrequencyServer;
      
      /** \brief Service server for querying the state of this worker
        */ 
      ros::ServiceServer getStateServer;
      
      /** \brief The worker's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The worker's cancellation condition
        */ 
      boost::condition cancelCondition;
      
      /** \brief The diagnostic task for monitoring the status of this
        *   worker
        */ 
      WorkerStatusTask statusTask;
      
      /** \brief The diagnostic task for monitoring the frequency
        *   of this worker
        */ 
      StatefulFrequencyTask frequencyTask;
    };
  };
};

#endif
