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

/** \file TimerQueueCallback.h
  * \brief Header file providing the TimerQueueCallback class interface
  */

#ifndef ROSCPP_NODEWRAP_TIMER_QUEUE_CALLBACK_H
#define ROSCPP_NODEWRAP_TIMER_QUEUE_CALLBACK_H

#include <ros/callback_queue_interface.h>

#include <roscpp_nodewrap/timer/TimerManager.h>

namespace nodewrap {
  /** \brief ROS timer queue callback
    * 
    * This class provides a callback for the timer which can be processed
    * by ROS callback queues.
    */
  class TimerQueueCallback :
    public ros::CallbackInterface {
  public:
    /** \brief Default constructor
      */
    TimerQueueCallback(const TimerManager::ImplPtr& manager, const
      TimerInfoPtr& timerInfo, const ros::Time& expectedTimeOfLastCallback,
      const ros::Time& actualTimeOfLastCallback, const ros::Time&
      expectedTimeOfCurrentCallback);
    
    /** \brief Destructor
      */
    ~TimerQueueCallback();

    /** \brief Call this timer queue callback.
      * 
      * \return The result of the call.
      */
    ros::CallbackInterface::CallResult call();
  private:
    /** \brief The timer manager owning this callback
      */
    TimerManager::ImplWPtr manager;
    
    /** \brief The timer information of the timer associated with this
      *   callback
      */
    TimerInfoWPtr timerInfo;
    
    /** \brief The expected time of the timer's last callback
      */ 
    ros::Time expectedTimeOfLastCallback;
    
    /** \brief The actual time of the timer's last callback
      */ 
    ros::Time actualTimeOfLastCallback;
    
    /** \brief The expected time of the timer's current callback
      */ 
    ros::Time expectedTimeOfCurrentCallback;

    /** \brief True, if the callback has been invoked
      */ 
    bool called;
  };
};

#endif
