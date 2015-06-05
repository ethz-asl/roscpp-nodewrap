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

/** \file WorkerQueueCallback.h
  * \brief Header file providing the WorkerQueueCallback class interface
  */

#ifndef ROSCPP_NODEWRAP_WORKER_QUEUE_CALLBACK_H
#define ROSCPP_NODEWRAP_WORKER_QUEUE_CALLBACK_H

#include <ros/callback_queue_interface.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS worker queue callback
    * 
    * This class provides a callback for the worker which can be processed
    * by ROS callback queues.
    */
  class WorkerQueueCallback :
    public ros::CallbackInterface {
  public:
    /** \brief Default constructor
      */
    WorkerQueueCallback(const WorkerQueueCallbackCallback& callback,
      const ros::VoidConstWPtr& trackedObject, bool hasTrackedObject);
    
    /** \brief Destructor
      */
    ~WorkerQueueCallback();

    /** \brief Call this worker queue callback.
      * 
      * \return The result of the call.
      */
    ros::CallbackInterface::CallResult call();
  private:
    /** \brief The worker queue callback's callback
      */
    WorkerQueueCallbackCallback callback;
    
    /** \brief A shared pointer to an object to track for the worker
      *   callbacks
      */ 
    ros::VoidConstWPtr trackedObject;
    
    /** \brief If true, the worker has an object to track for its
      *   callbacks
      */ 
    bool hasTrackedObject;    
  };
};

#endif
