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

/** \file TimerInfo.h
  * \brief Header file providing the TimerInfo class interface
  */

#ifndef ROSCPP_NODEWRAP_TIMER_INFO_H
#define ROSCPP_NODEWRAP_TIMER_INFO_H

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS high precision timer information
    * 
    * This class is a helper class to represent the information associated
    * with a high precision timer.
    */
  class TimerInfo {
  friend class TimerManager;
  friend class TimerQueueCallback;
  public:
    /** \brief Default constructor
      */
    TimerInfo();

  private:
    /** \brief The timer's handle
      */ 
    int handle;
    
    /** \brief The timer's period
      */ 
    ros::Duration period;

    /** \brief The timer's callback
      */ 
    ros::TimerCallback callback;
    
    /** \brief The timer's callback queue
      */ 
    ros::CallbackQueueInterface* callbackQueue;

    /** \brief The timer's last callback duration
      */ 
    ros::WallDuration durationOfLastCallback;

    /** \brief The expected time of the timer's last callback
      */ 
    ros::Time expectedTimeOfLastCallback;
    
    /** \brief The expected time of the timer's next callback
      */ 
    ros::Time expectedTimeOfNextCallback;

    /** \brief The actual time of the timer's last callback
      */ 
    ros::Time actualTimeOfLastCallback;

    /** \brief If true, the timer has been removed
      */ 
    bool removed;

    /** \brief The object tracked for the timer's callbacks
      */ 
    ros::VoidConstWPtr trackedObject;
    
    /** \brief True, if the timer has an object to track for its callbacks
      */ 
    bool hasTrackedObject;

    /** \brief The timer's number of waiting callbacks
      */ 
    size_t numWaitingCallbacks;

    /** \brief If true, the timer is non-cyclic
      */ 
    bool oneshot;

    /** \brief The timer's total number of callback calls
      */ 
    size_t totalNumCalls;
    
    /** \brief The mutex guarding this timer information
      */ 
    boost::mutex mutex;
    
  };
};

#endif
