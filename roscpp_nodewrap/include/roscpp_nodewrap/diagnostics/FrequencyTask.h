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

/** \file FrequencyTask.h
  * \brief Header file providing the FrequencyTask class interface
  */

#ifndef ROSCPP_NODEWRAP_FREQUENCY_TASK_H
#define ROSCPP_NODEWRAP_FREQUENCY_TASK_H

#include <boost/thread/locks.hpp>

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/FrequencyTaskOptions.h>

#include <roscpp_nodewrap/statistics/FrequencyStatistics.h>

namespace nodewrap {
  /** \brief Diagnostic task for frequency monitoring
    * 
    * This class provides a diagnostic task to monitor the frequency of
    * a cyclic event.
    */
  class FrequencyTask :
    public DiagnosticTask {
  friend class DiagnosticUpdater;
  public:
    /** \brief Forward declaration of the frequency task options
      */
    typedef FrequencyTaskOptions Options;
    
    /** \brief Default constructor
      */
    FrequencyTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source frequency task which is being copied
      *   to this frequency task.
      */
    FrequencyTask(const FrequencyTask& src);
    
    /** \brief Destructor
      */
    ~FrequencyTask();

    /** \brief Retrieve the frequency statistics of this diagnostic task
      */
    FrequencyStatistics getStatistics() const;
    
    /** \brief Retrieve the frequency statistics' estimates of this
      *   diagnostic task
      */
    FrequencyStatistics::Estimates getStatisticsEstimates() const;
    
    /** \brief Start this diagnostic task
      */
    void start();
    
    /** \brief Signal this diagnostic task that the monitored event
      *   has occurred
      * 
      * \param[in] timeOfEvent The time of the event.
      */
    inline void event(const ros::Time& timeOfEvent = ros::Time::now()) {
      if (impl)
        impl->event(timeOfEvent);
    };
    
    /** \brief Stop this diagnostic task
      */
    void stop();
    
  private:
    /** \brief ROS frequency task implementation
      * 
      * This class provides the private implementation of the frequency
      * task.
      */
    class Impl :
      public DiagnosticTask::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const std::string& name, const Options& defaultOptions, const
        NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Retrieve the frequency statistics of this diagnostic task
        *   (implementation)
        */
      FrequencyStatistics getStatistics() const;
      
      /** \brief Retrieve the frequency statistics' estimates of this
        *   diagnostic task (implementation)
        */
      FrequencyStatistics::Estimates getStatisticsEstimates() const;
      
      /** \brief Start this diagnostic task (implementation)
        */
      void start();
      
      /** \brief Signal this diagnostic task that the monitored event
        *   has occurred (implementation)
        */
      inline void event(const ros::Time& timeOfEvent = ros::Time::now()) {
        boost::mutex::scoped_lock lock(mutex);
        
        statistics.event(timeOfEvent);
      };
      
      /** \brief Stop this diagnostic task (implementation)
        */
      void stop();
      
      /** \brief Fill out this frequency task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
      
      /** \brief The frequency statistics of this diagnostic task
        */ 
      FrequencyStatistics statistics;

      /** \brief The window of the frequency task's statistics
        */
      ros::Duration window;
    
      /** \brief The expected mean frequency to be diagnosed
        */
      double expected;
      
      /** \brief The warning tolerance of the diagnosed mean frequency
        */
      double warnMeanTolerance;
      
      /** \brief The error tolerance of the diagnosed mean frequency
        */
      double errorMeanTolerance;
      
      /** \brief The warning tolerance of the diagnosed frequency standard
        *   deviation
        */
      double warnStandardDeviationTolerance;
      
      /** \brief The error tolerance of the diagnosed frequency standard
        *   deviation
        */
      double errorStandardDeviationTolerance;
      
      /** \brief True, if this frequency task has been started
        */
      bool started;
    };
      
    /** \brief Declaration of the frequency task implementation
      *   pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the frequency task implementation
      *   weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The frequency task's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    FrequencyTask(const std::string& name, const Options& defaultOptions,
      const NodeImplPtr& nodeImpl);
  };
};

#endif
