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

/** \file WorkerEvent.h
  * \brief Header file providing the WorkerEvent class interface
  */

#ifndef ROSCPP_NODEWRAP_WORKER_EVENT_H
#define ROSCPP_NODEWRAP_WORKER_EVENT_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS worker event
    * 
    * This class is passed as a parameter to the worker event.
    */
  class WorkerEvent {
  public:
    /** \brief Default constructor
      */
    WorkerEvent();

    /** \brief True, if the worker owning this worker event has been
      *   canceled
      */
    bool isWorkerCanceled() const;
    
    /** \brief The expected cycle time of the worker
      */ 
    ros::Duration expectedCycleTime;
    
    /** \brief The momentary, actual cycle time of the worker
      */ 
    ros::Duration actualCycleTime;
    
  private:
    friend class Worker;
    
    /** \brief The worker owning this worker event
      */
    WorkerImplPtr worker;
  };
};

#endif
