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

/** \file PeriodTask.h
  * \brief Header file providing the PeriodTask class interface
  */

#ifndef ROSCPP_NODEWRAP_PERIOD_TASK_H
#define ROSCPP_NODEWRAP_PERIOD_TASK_H

#include <roscpp_nodewrap/statistics/PeriodStatistics.h>

#include <roscpp_nodewrap/diagnostics/CyclicEventTask.h>
#include <roscpp_nodewrap/diagnostics/PeriodTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic task for period monitoring
    * 
    * This class provides the basis of all diagnostic tasks which monitor
    * the period of a cyclic event.
    */
  class PeriodTask :
    public CyclicEventTask<PeriodStatistics> {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Forward declaration of the period task options
      */
    typedef PeriodTaskOptions Options;
    
    /** \brief Default constructor
      */
    PeriodTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source period task which is being copied
      *   to this period task.
      */
    PeriodTask(const PeriodTask& src);
    
    /** \brief Destructor
      */
    ~PeriodTask();
    
  protected:
    /** \brief ROS period task implementation
      * 
      * This class provides the private implementation of the period
      * task.
      */
    class Impl :
      public CyclicEventTask<PeriodStatistics>::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Retrieve the expected time of the next event
        *   (implementation)
        */
      ros::Time getExpectedTimeOfNextEvent() const;
    };
  };
};

#endif
