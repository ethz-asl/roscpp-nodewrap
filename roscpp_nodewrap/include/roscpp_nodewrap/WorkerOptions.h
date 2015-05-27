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

/** \file WorkerOptions.h
  * \brief Header file providing the WorkerOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_WORKER_OPTIONS_H
#define ROSCPP_NODEWRAP_WORKER_OPTIONS_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS worker options
    * 
    * This class encapsulates all options available for creating a
    * timer-controlled node worker.
    */
  class WorkerOptions {
  public:
    /** \brief Default constructor
      */
    WorkerOptions();

    /** \brief The name of the new worker, a valid ROS graph resource name
      */ 
    std::string name;
    
    /** \brief The rate at which the worker's work callback will be attempted
      *   to be invoked by the worker's timer
      */ 
    ros::Rate rate;
    
    /** \brief If true, the worker will be started automatically
      */ 
    bool autostart;
    
    /** \brief A function to call when the worker should perform its work
      */ 
    WorkerCallback callback;
    
    /** \brief The callback queue to be used by the worker's timer
      */ 
    ros::CallbackQueueInterface* callbackQueue;
    
    /** \brief A shared pointer to an object to track for the worker's
      *   timer callbacks
      */ 
    ros::VoidConstPtr trackedObject;    
  };
};

#endif
