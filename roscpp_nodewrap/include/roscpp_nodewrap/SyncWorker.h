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
    * This class provides a timer-controlled worker for use with the ROS
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
    SyncWorker(const Worker& src);
    
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
      
      /** \brief Start the worker
        */
      void start();
            
      /** \brief Cancel the worker
        */
      void cancel(bool block = false);
            
      /** \brief Unadvertise the worker's services
        */
      void unadvertise();
            
      /** \brief The timer callback of this worker
        */ 
      void timerCallback(const ros::TimerEvent& timerEvent);
      
      /** \brief The timer controlling this worker
        */ 
      ros::Timer timer;      
    };
    
    /** \brief Constructor (private version)
      */
    SyncWorker(const std::string& name, const WorkerOptions& defaultOptions,
      const NodeImplPtr& nodeImpl);
  };
};

#endif
