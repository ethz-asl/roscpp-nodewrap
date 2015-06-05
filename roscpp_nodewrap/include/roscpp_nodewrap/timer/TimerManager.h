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

/** \file TimerManager.h
  * \brief Header file providing the TimerManager class interface
  */

#ifndef ROSCPP_NODEWRAP_TIMER_MANAGER_H
#define ROSCPP_NODEWRAP_TIMER_MANAGER_H

#include <list>
#include <vector>

#include <ros/ros.h>

#include <boost/enable_shared_from_this.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <roscpp_nodewrap/timer/Timer.h>

namespace nodewrap {
  /** \brief ROS high precision timer manager
    * 
    * This class provides the facilities for managing a collection of
    * high precision timers.
    */
  class TimerManager {
  friend class NodeImpl;
  friend class Timer;
  friend class TimerQueueCallback;
  public:
    /** \brief Default constructor
      */
    TimerManager();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source timer manager which is being copied to
      *   this timer manager.
      */
    TimerManager(const TimerManager& src);
    
    /** \brief Destructor
      */
    ~TimerManager();
    
    /** \brief Perform shutdown of the timer manager
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return impl ? (void*)1 : (void*)0;
    };
    
  private:    
    /** \brief ROS high precision timer manager implementation
      * 
      * This class provides the private implementation of the high
      * precision time manager.
      */
    class Impl :
      public boost::enable_shared_from_this<Impl> {
    public:
      /** \brief Default constructor
        */
      Impl(const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Set the period of the referred timer
        */
      void setPeriod(int timerHandle, const ros::Duration& period);
      
      /** \brief True, if the referred timer has pending callbacks
        */
      bool hasPending(int timerHandle);
    
      /** \brief Add a timer
        */
      int addTimer(const ros::Duration& period, const ros::TimerCallback&
        callback, ros::CallbackQueueInterface* callbackQueue, const
        ros::VoidConstPtr& trackedObject, bool oneshot);
      
      /** \brief Remove the referred timer
        */
      void removeTimer(int timerHandle);
      
      /** \brief Perform shutdown of the timer manager (implementation)
        */
      void shutdown();
      
      /** \brief Compare two timers with respect to their callback times
        */ 
      bool waitingCompare(int leftTimerHandle, int rightTimerHandle);
      
      /** \brief Find timer by handle
        */ 
      TimerInfoPtr findTimer(int timerHandle);
      
      /** \brief Schedule timer callback
        */ 
      void schedule(const TimerInfoPtr& timerInfo);
      
      /** \brief Update the expected time of a timer's next callback
        */ 
      void updateNext(const TimerInfoPtr& timerInfo, const ros::Time& now);
      
      /** \brief Spin over pending timer callbacks
        */ 
      void spin();

      /** \brief The timer informations of the managed timers
        */ 
      std::vector<TimerInfoPtr> timerInfos;
      
      /** \brief The mutex guarding the manager's timers
        */ 
      boost::mutex timersMutex;
      
      /** \brief The condition for signaling timer events
        */ 
      boost::condition_variable timersCondition;
      
      /** \brief True, if a new timer is awaiting scheduling
        */ 
      volatile bool newTimer;

      /** \brief The mutex guarding the timer wait operations
        */ 
      boost::mutex waitingMutex;
      
      /** \brief The handles of the waiting timer
        */ 
      std::list<int> waitingTimers;

      /** \brief The manager's number of timer handles
        */ 
      size_t numHandles;
      
      /** \brief The mutex guarding the manager's timer handles
        */ 
      boost::mutex handlesMutex;

      /** \brief The manager's spinner
        */ 
      boost::thread spinner;

      /** \brief If true, the manager's spinner has been started
        */ 
      bool started;

      /** \brief If true, the manager's spinner has been canceled
        */ 
      bool canceled;
      
      /** \brief The node implementation owning this timer manager
        */ 
      NodeImplPtr nodeImpl;
    };
    
    /** \brief Declaration of the timer manager implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the timer manager implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The timer manager's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    TimerManager(const NodeImplPtr& nodeImpl);
    
    /** \brief Add a timer to be managed by this timer manager
      */
    Timer addTimer(const ros::TimerOptions& options);
  };
};

#endif
