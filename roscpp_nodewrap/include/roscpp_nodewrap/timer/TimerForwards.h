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

/** \file TimerForwards.h
  * \brief Header file providing forward declarations for the node timer
  */

#ifndef ROSCPP_NODEWRAP_TIMER_FORWARDS_H
#define ROSCPP_NODEWRAP_TIMER_FORWARDS_H

#include <ros/ros.h>

namespace nodewrap {
  /** \brief Forward declaration of thetimer
    */
  class Timer;
  /** \brief Forward declaration of the timer pointer type
    */
  typedef boost::shared_ptr<Timer> TimerPtr;
  /** \brief Forward declaration of the timer weak pointer type
    */
  typedef boost::weak_ptr<Timer> TimerWPtr;

  /** \brief Forward declaration of the timer information
    */
  class TimerInfo;
  /** \brief Declaration of the timer information pointer type
    */
  typedef boost::shared_ptr<TimerInfo> TimerInfoPtr;
  
  /** \brief Declaration of the timer information weak pointer type
    */
  typedef boost::weak_ptr<TimerInfo> TimerInfoWPtr;
  
  /** \brief Forward declaration of the timer manager
    */
  class TimerManager;
  /** \brief Forward declaration of the timer manager pointer type
    */
  typedef boost::shared_ptr<TimerManager> TimerManagerPtr;
  /** \brief Forward declaration of the timer manager weak pointer type
    */
  typedef boost::weak_ptr<TimerManager> TimerManagerWPtr;
  
  /** \brief Forward declaration of the timer queue callback
    */
  class TimerQueueCallback;
};

#endif
