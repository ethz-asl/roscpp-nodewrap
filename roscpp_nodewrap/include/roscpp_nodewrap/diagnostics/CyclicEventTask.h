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

/** \file CyclicEventTask.h
  * \brief Header file providing the CyclicEventTask class interface
  */

#ifndef ROSCPP_NODEWRAP_CYCLIC_EVENT_TASK_H
#define ROSCPP_NODEWRAP_CYCLIC_EVENT_TASK_H

#include <roscpp_nodewrap/diagnostics/StatisticsTask.h>

namespace nodewrap {
  /** \brief Diagnostic task for monitoring a cyclic event
    * 
    * This class provides the basis of all diagnostic tasks which monitor
    * the timing of a cyclic event.
    */
  template <class S> class CyclicEventTask :
    public StatisticsTask<S> {
  public:
    /** \brief Forward declaration of the cyclic event task options
      */
    typedef typename StatisticsTask<S>::Options Options;
    
    /** \brief Default constructor
      */
    CyclicEventTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source cyclic event task which is being copied
      *   to this cyclic event task.
      */
    CyclicEventTask(const CyclicEventTask& src);
    
    /** \brief Destructor
      */
    ~CyclicEventTask();
    
    /** \brief Retrieve the expected time of the next event
      */
    ros::Time getExpectedTimeOfNextEvent() const;
      
    /** \brief Update the statistics of this diagnostic task with the time
      *   of occurrence of an event
      * 
      * \param[in] timeOfEvent The time of occurrence of the event.
      */
    void event(const ros::Time& timeOfEvent = ros::Time::now());
    
  protected:
    /** \brief ROS cyclic event task implementation
      * 
      * This class provides the private implementation of the cyclic event
      * task.
      */
    class Impl :
      public StatisticsTask<S>::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        typename StatisticsTask<S>::ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Retrieve the expected time of the next event
        *   (abstract implementation)
        */
      virtual ros::Time getExpectedTimeOfNextEvent() const = 0;
      
      /** \brief Update the statistics of this diagnostic task with the time
        *   of occurrence of an event (implementation)
        */
      void event(const ros::Time& timeOfEvent = ros::Time::now());
            
      /** \brief Fill out this frequency task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
      
      /** \brief Fill out this frequency task's status summary using
        *   its estimates
        */
      void statusSummary(diagnostic_updater::DiagnosticStatusWrapper& status,
        const typename S::Estimates& estimates);
      
      /** \brief Add key-value pairs to this frequency task's status
        *   using its estimates
        */
      void statusAdd(diagnostic_updater::DiagnosticStatusWrapper& status,
        const typename S::Estimates& estimates);
    };
  };
};

#include <roscpp_nodewrap/diagnostics/CyclicEventTask.tpp>

#endif
