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

/** \file LatencyTask.h
  * \brief Header file providing the LatencyTask class interface
  */

#ifndef ROSCPP_NODEWRAP_LATENCY_TASK_H
#define ROSCPP_NODEWRAP_LATENCY_TASK_H

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include <roscpp_nodewrap/statistics/LatencyStatistics.h>

#include <roscpp_nodewrap/diagnostics/LatencyTaskOptions.h>
#include <roscpp_nodewrap/diagnostics/StatisticsTask.h>

namespace nodewrap {
  /** \brief Diagnostic task for latency monitoring
    * 
    * This class provides the basis of all diagnostic tasks which monitor
    * the time difference between two events.
    */
  class LatencyTask :
    public StatisticsTask<LatencyStatistics> {
  public:
    /** \brief Forward declaration of the latency task options
      */
    typedef LatencyTaskOptions Options;
    
    /** \brief Default constructor
      */
    LatencyTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source latency task which is being copied
      *   to this latency task.
      */
    LatencyTask(const LatencyTask& src);
    
    /** \brief Destructor
      */
    ~LatencyTask();
      
    /** \brief Update the statistics of this diagnostic task with the time
      *   of occurrence of two events
      * 
      * \param[in] timeOfFirstEvent The time of occurrence of the first
      *   event.
      * \param[in] timeOfSecondEvent The time of occurrence of the second
      *   event.
      */
    inline void events(const ros::Time& timeOfFirstEvent, const ros::Time&
        timeOfSecondEvent = ros::Time::now()) {
      if (impl)
        impl->as<LatencyTask::Impl>().events(timeOfFirstEvent,
          timeOfSecondEvent);
    };
    
  protected:
    /** \brief ROS latency task implementation
      * 
      * This class provides the private implementation of the latency
      * task.
      */
    class Impl :
      public StatisticsTask<LatencyStatistics>::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Update the statistics of this diagnostic task with the time
        *   of occurrence of two events (implementation)
        */
      inline void events(const ros::Time& timeOfFirstEvent, const ros::Time&
          timeOfSecondEvent = ros::Time::now()) {
        boost::mutex::scoped_lock lock(mutex);
        
        statistics.events(timeOfFirstEvent, timeOfSecondEvent);
      };
            
      /** \brief Fill out this latency task's status summary using
        *   its estimates
        */
      void statusSummary(diagnostic_updater::DiagnosticStatusWrapper& status,
        const LatencyStatistics::Estimates& estimates);
      
      /** \brief Add key-value pairs to this latency task's status
        *   using its estimates
        */
      void statusAdd(diagnostic_updater::DiagnosticStatusWrapper& status,
        const LatencyStatistics::Estimates& estimates);
    };
  };
};

#endif
