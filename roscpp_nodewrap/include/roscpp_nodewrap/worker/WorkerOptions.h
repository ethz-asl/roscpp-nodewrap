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

#include <roscpp_nodewrap/diagnostics/FrequencyTaskOptions.h>
#include <roscpp_nodewrap/diagnostics/WorkerStatusTaskOptions.h>

namespace nodewrap {
  /** \brief ROS worker options
    * 
    * This class encapsulates all options available for creating a
    * node worker.
    */
  class WorkerOptions {
  public:
    /** \brief Default constructor
      */
    WorkerOptions();

    /** \brief The namespace of the worker options
      */
    std::string ns;
    
    /** \brief The frequency at which the worker's callback is expected to
      *   be invoked
      * 
      * \note A frequency of zero states that the worker is non-cyclic
      *   a should only be run once.
      */ 
    double frequency;
    
    /** \brief If true, the worker will be started automatically
      */ 
    bool autostart;
    
    /** \brief If true, the worker will be synchronous
      */ 
    bool synchronous;
    
    /** \brief The duration of the window for the worker statistics
      */ 
    ros::Duration statisticsWindow;
    
    /** \brief A function to call when the worker should perform its work
      */ 
    WorkerCallback callback;
    
    /** \brief If true, the worker uses a private callback queue
      * 
      * \note This parameter will only take effect if the callback queue is
      *   explicitly specified by these options.
      */ 
    bool privateCallbackQueue;
    
    /** \brief The priority of the worker as interpreted by the operating
      *   system scheduler
      * 
      * \note This parameter will only take effect if the worker uses
      *   a private callback queue as specified by theses options.
      */ 
    int priority;
    
    /** \brief The callback queue to be used by the worker
      * 
      * \note If null, the worker will instantiate and use a private
      *   callback queue if specified by these options.
      */ 
    ros::CallbackQueueInterface* callbackQueue;
    
    /** \brief A shared pointer to an object to track for the worker
      *   callbacks
      */ 
    ros::VoidConstPtr trackedObject;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   status of the worker
      */ 
    WorkerStatusTaskOptions statusTaskOptions;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   frequency of the worker
      */ 
    FrequencyTaskOptions frequencyTaskOptions;
  };
};

#endif
