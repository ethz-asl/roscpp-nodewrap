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

/** \file StatisticsTask.h
  * \brief Header file providing the StatisticsTask class interface
  */

#ifndef ROSCPP_NODEWRAP_STATISTICS_TASK_H
#define ROSCPP_NODEWRAP_STATISTICS_TASK_H

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/StatisticsTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic task for statistics monitoring
    * 
    * This templated class provides the basis of all diagnostic tasks
    * which monitor the estimates of some statistics.
    */
  template <class S> class StatisticsTask :
    public DiagnosticTask {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Forward declaration of the statistics task options
      */
    typedef StatisticsTaskOptions<S> Options;
    
    /** \brief Default constructor
      */
    StatisticsTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source statistics task which is being copied
      *   to this statistics task.
      */
    StatisticsTask(const StatisticsTask<S>& src);
    
    /** \brief Destructor
      */
    ~StatisticsTask();

    /** \brief Retrieve the statistics of this diagnostic task
      */
    S getStatistics() const;
    
    /** \brief Retrieve the statistics' estimates of this diagnostic
      *   task
      */
    typename S::Estimates getStatisticsEstimates() const;
    
    /** \brief Update the statistics of this diagnostic task
      */
    void update(const typename S::Variate& sample);
    
    /** \brief Clear the statistics of this diagnostic task
      */
    void clear();
    
  protected:
    /** \brief ROS statistics task implementation
      * 
      * This class provides the private implementation of the statistics
      * task.
      */
    class Impl :
      public DiagnosticTask::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name,
        const ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Retrieve the statistics of this diagnostic task
        *   (implementation)
        */
      const S& getStatistics() const;
      
      /** \brief Retrieve the statistics' estimates of this diagnostic
        *   task (implementation)
        */
      typename S::Estimates getStatisticsEstimates() const;
      
      /** \brief Update the statistics of this diagnostic task
        *   (implementation)
        */
      virtual void update(const typename S::Variate& sample);
    
      /** \brief Stop this statistics task (implementation)
        */
      void stop();
      
      /** \brief Clear the statistics of this statistics task (implementation)
        */
      void clear();
          
      /** \brief Fill out this statistics task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
      
      /** \brief Fill out this statistics task's status summary using
        *   its estimates
        */
      virtual void statusSummary(diagnostic_updater::DiagnosticStatusWrapper&
        status, const typename S::Estimates& estimates);
      
      /** \brief Add key-value pairs to this statistics task's status
        *   using its estimates
        */
      virtual void statusAdd(diagnostic_updater::DiagnosticStatusWrapper&
        status, const typename S::Estimates& estimates);
      
      /** \brief The statistics of this diagnostic task
        */ 
      S statistics;

      /** \brief The window size of the task's statistics
        */
      size_t windowSize;
    
      /** \brief The expected value to be diagnosed
        */
      typename S::Variate expected;
      
      /** \brief The warning tolerance of the diagnosed mean
        */
      double warnMeanTolerance;
      
      /** \brief The error tolerance of the diagnosed mean
        */
      double errorMeanTolerance;
      
      /** \brief The warning tolerance of the diagnosed standard
        *   deviation
        */
      double warnStandardDeviationTolerance;
      
      /** \brief The error tolerance of the diagnosed standard
        *   deviation
        */
      double errorStandardDeviationTolerance;
      
      /** \brief The statistics task's mutex
        */ 
      mutable boost::mutex mutex;
    };
  };
};

#include <roscpp_nodewrap/diagnostics/StatisticsTask.tpp>

#endif
