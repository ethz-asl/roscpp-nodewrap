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

#include <roscpp_nodewrap/Manager.h>

#include <roscpp_nodewrap/timer/Timer.h>

namespace nodewrap {
  /** \brief ROS high precision timer manager
    * 
    * This class provides the facilities for managing a collection of
    * high precision timers.
    */
  class TimerManager :
    public Manager<Timer, int> {
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
        
    /** \brief Add a timer to be managed by this timer manager
      */
    Timer addTimer(const ros::TimerOptions& options);
    
  private:    
    /** \brief ROS high precision timer manager implementation
      * 
      * This class provides the private implementation of the high
      * precision time manager.
      */
    class Impl :
      public Manager<Timer, int>::Impl {
    public:
      /** \brief Default constructor
        */
      Impl(const NodeImplPtr& node);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Retrieve the node owning this timer manager
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief Set the period of the referred timer
        */
      void setTimerPeriod(int handle, const ros::Duration& period);
      
      /** \brief True, if the referred timer has pending callbacks
        */
      bool timerHasPending(int handle);
    
      /** \brief Start the referred timer
        */
      void startTimer(int handle);
      
      /** \brief Stop the referred timer
        */
      void stopTimer(int handle);
      
      /** \brief Perform shutdown of the timer manager (implementation)
        */
      void shutdown();
      
      /** \brief Compare two waiting timers with respect to their absolute
        *   callback times
        */ 
      bool compareWaitingTimers(int leftHandle, int rightHandle);
      
      /** \brief Find the referred timer
        */ 
      TimerInfoPtr findTimer(int handle);
      
      /** \brief Schedule timer callback
        */ 
      void scheduleTimerCallback(const TimerInfoPtr& timerInfo);
      
      /** \brief Update the expected time of a timer's next callback
        */ 
      void updateNextTimerCallback(const TimerInfoPtr& timerInfo, const
        ros::Time& now);
      
      /** \brief Spin over pending timer callbacks
        */ 
      void spin();

      /** \brief The timers managed by this timer manager
        */ 
      std::list<TimerInfoPtr> timers;

      /** \brief The number of timers managed by this timer manager
        */ 
      size_t numTimers;
      
      /** \brief The mutex guarding the manager's timers
        */ 
      boost::mutex timerMutex;
      
      /** \brief The condition for signaling timer events
        */ 
      boost::condition_variable timerCondition;
      
      /** \brief True, if a new timer is awaiting scheduling
        */ 
      volatile bool newTimer;

      /** \brief The mutex guarding the waiting timers
        */ 
      boost::mutex waitingQueueMutex;
      
      /** \brief The waiting timers queued by handle
        */ 
      std::list<int> waitingQueue;

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
      NodeImplPtr node;
    };
    
    /** \brief Constructor (private version)
      */
    TimerManager(const NodeImplPtr& node);
  };
};

#endif
