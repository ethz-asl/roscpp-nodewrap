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

/** \file Timer.h
  * \brief Header file providing the Timer class interface
  */

#ifndef ROSCPP_NODEWRAP_TIMER_H
#define ROSCPP_NODEWRAP_TIMER_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS high precision timer
    * 
    * This class defines a high precision timer for use with the ROS
    * node implementation. Its interface is identical to that of the
    * ROS standard timer, but its implementation features microsecond
    * precision.
    */
  class Timer {
  friend class TimerManager;
  public:
    /** \brief Default constructor
      */
    Timer();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source timer which is being copied to this
      * timer.
      */
    Timer(const Timer& src);
    
    /** \brief Destructor
      */
    ~Timer();
    
    /** \brief Set the period of this timer
      */
    void setPeriod(const ros::Duration& period);
    
    /** \brief True, if this timer has pending events
      */
    bool hasPending();
    
    /** \brief Start this timer
      */
    void start();
      
    /** \brief Stop this timer
      */
    void stop();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const Timer& timer) const {
      return (impl < timer.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const Timer& timer) const {
      return (impl == timer.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const Timer& timer) const {
      return (impl != timer.impl);
    };
    
  private:
    /** \brief ROS high precision timer implementation
      * 
      * This class provides the private implementation of the high
      * precision timer.
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(const ros::TimerOptions& options, const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Set the period of this timer implementation
        */
      void setPeriod(const ros::Duration& period);
      
      /** \brief True, if this timer implementation has pending events
        */
      bool hasPending();
      
      /** \brief True, if this timer implementation is valid
        */
      bool isValid() const;
      
      /** \brief Start this timer (implementation)
        */
      void start();
        
      /** \brief Stop this timer (implementation)
        */
      void stop();
      
      /** \brief True, if this timer has been started
        */ 
      bool started;
      
      /** \brief The timer's internal handle
        */ 
      int handle;
      
      /** \brief The timer's period
        */ 
      ros::Duration period;
      
      /** \brief True, if the timer will be started automatically
        */ 
      bool autostart;
      
      /** \brief True, if this timer is non-cyclic
        */ 
      bool oneshot;
      
      /** \brief The timer's callback
        */ 
      ros::TimerCallback callback;
      
      /** \brief The callback queue used by this timer
        */ 
      ros::CallbackQueueInterface* callbackQueue;
      
      /** \brief A shared pointer to an object being tracked for the
        *   timer callbacks
        */ 
      ros::VoidConstWPtr trackedObject;
      
      /** \brief If true, the timer has an object to track for its
        *   callbacks
        */ 
      bool hasTrackedObject;
      
      /** \brief The node implementation owning this timer
        */ 
      NodeImplPtr nodeImpl;
    };

    /** \brief Declaration of the timer implementation pointer
      *   type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the timer implementation weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The timer's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    Timer(const ros::TimerOptions& options, const NodeImplPtr& nodeImpl);
  };
};

#endif
