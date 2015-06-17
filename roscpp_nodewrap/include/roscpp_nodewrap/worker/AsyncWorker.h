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

/** \file AsyncWorker.h
  * \brief Header file providing the AsyncWorker class interface
  */

#ifndef ROSCPP_NODEWRAP_ASYNC_WORKER_H
#define ROSCPP_NODEWRAP_ASYNC_WORKER_H

#include <ros/callback_queue.h>

#include <roscpp_nodewrap/worker/Worker.h>

namespace nodewrap {
  /** \brief ROS asynchronous node worker
    * 
    * This class provides a high precision timer-controlled worker for use
    * with the ROS node implementation.
    */
  class AsyncWorker :
    public Worker {
  friend class WorkerManager;
  friend class WorkerStatusTask;
  public:
    /** \brief Default constructor
      */
    AsyncWorker();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source asynchronous worker which is being copied
      *   to this asynchronous worker.
      */
    AsyncWorker(const AsyncWorker& src);
    
    /** \brief Destructor
      */
    ~AsyncWorker();
    
  private:
    /** \brief ROS asynchronous node worker implementation
      * 
      * This class provides the private implementation of the asynchronous
      * node worker.
      */
    class Impl :
      public Worker::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Initialize the asynchronous worker
        */
      void init(const WorkerOptions& defaultOptions);
      
      /** \brief Start the worker (thread-safe implementation)
        */
      void safeStart();
            
      /** \brief Wake the worker (thread-safe implementation)
        */
      void safeWake();
            
      /** \brief Stop the worker (thread-safe implementation)
        */
      void safeStop();
            
      /** \brief The timer callback of this worker
        */ 
      void timerCallback(const ros::TimerEvent& timerEvent);
      
      /** \brief The high precision timer controlling this worker
        */ 
      Timer timer;
      
      /** \brief If true, the timer controlling this worker needs to
        *   be reset
        */ 
      bool resetTimer;
    };
  };
};

#endif
