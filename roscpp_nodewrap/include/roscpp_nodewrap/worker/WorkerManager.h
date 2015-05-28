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

/** \file WorkerManager.h
  * \brief Header file providing the WorkerManager class interface
  */

#ifndef ROSCPP_NODEWRAP_WORKER_MANAGER_H
#define ROSCPP_NODEWRAP_WORKER_MANAGER_H

#include <ros/ros.h>
#include <ros/timer.h>

#include <boost/thread/mutex.hpp>

#include <roscpp_nodewrap/Forwards.h>

#include <roscpp_nodewrap/worker/Worker.h>
#include <roscpp_nodewrap/worker/WorkerOptions.h>

#include <roscpp_nodewrap_msgs/HasWorker.h>
#include <roscpp_nodewrap_msgs/ListWorkers.h>

namespace nodewrap {
  /** \brief ROS node worker manager
    * 
    * This class provides the facilities for managing a collection of
    * node workers.
    */
  class WorkerManager {
  friend class NodeImpl;
  public:
    /** \brief Default constructor
      */
    WorkerManager();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source worker manager which is being copied to
      *   this worker manager.
      */
    WorkerManager(const WorkerManager& src);
    
    /** \brief Destructor
      */
    ~WorkerManager();
    
    /** \brief Perform shutdown of the worker manager
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
  private:
    /** \brief ROS node worker manager implementation
      * 
      * This class provides the private implementation of the node worker
      * manager.
      */
    class Impl {
    public:
      /** \brief Default constructor
        */
      Impl(const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Query if this worker manager is valid
        */
      bool isValid() const;
      
      /** \brief Unadvertise the worker manager's services
        */
      void unadvertise();
      
      /** \brief Service callback listing the workers managed by this
        *   worker manager
        */ 
      bool listWorkersCallback(ListWorkers::Request& request,
        ListWorkers::Response& response);
      
      /** \brief Service callback for querying if a specific worker is managed
        *   by this worker manager
        */ 
      bool hasWorkerCallback(HasWorker::Request& request, HasWorker::Response&
        response);
      
      /** \brief Service server listing the workers managed by this
        *   worker manager
        */ 
      ros::ServiceServer listWorkersServer;
      
      /** \brief Service server for querying if a specific worker is managed
        *   by this worker manager
        */ 
      ros::ServiceServer hasWorkerServer;
      
      /** \brief The node implementation owning this worker manager
        */ 
      NodeImplPtr nodeImpl;
      
      /** \brief The worker manager's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The workers managed by this worker manager
        */ 
      std::map<std::string, Worker::ImplWPtr> workers;
    };
    
    /** \brief Declaration of the worker manager implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the worker manager implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The worker manager's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    WorkerManager(const NodeImplPtr& nodeImpl);
    
    /** \brief Add a worker to be managed by this worker manager
      */
    Worker addWorker(const std::string& name, const WorkerOptions& options);
  };
};

#endif
