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

/** \file SyncWorker.h
  * \brief Header file providing the SyncWorker class interface
  */

#ifndef ROSCPP_NODEWRAP_SYNCWORKER_H
#define ROSCPP_NODEWRAP_SYNCWORKER_H

#include <roscpp_nodewrap/Worker.h>

namespace nodewrap {
  /** \brief ROS synchronous node worker
    * 
    * This class provides an event-controlled worker for use with the ROS
    * node implementation.
    */
  class SyncWorker :
    public Worker {
  friend class WorkerManager;
  public:
    /** \brief Default constructor
      */
    SyncWorker();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source synchronous worker which is being copied
      *   to this synchronous worker.
      */
    SyncWorker(const SyncWorker& src);
    
    /** \brief Destructor
      */
    ~SyncWorker();
    
  private:
    /** \brief ROS synchronous node worker implementation
      * 
      * This class provides the private implementation of the synchronous
      * node worker.
      */
    class Impl :
      public Worker::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const WorkerOptions& defaultOptions,
        const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Start the worker (thread-safe implementation)
        */
      void safeStart();
            
      /** \brief Wake the worker (thread-safe implementation)
        */
      void safeWake();
            
      /** \brief Stop the worker (thread-safe implementation)
        */
      void safeStop();
      
      /** \brief The callback queue to be used by the worker
        */ 
      ros::CallbackQueueInterface* callbackQueue;
    
      /** \brief A shared pointer to an object to track for the worker
        *   callbacks
        */ 
      ros::VoidConstWPtr trackedObject;
      
      /** \brief If true, the worker has an object to track for its
        *   callbacks
        */ 
      bool hasTrackedObject;    
    };
    
    /** \brief Constructor (private version)
      */
    SyncWorker(const std::string& name, const WorkerOptions& defaultOptions,
      const NodeImplPtr& nodeImpl);
  };
};

#endif
