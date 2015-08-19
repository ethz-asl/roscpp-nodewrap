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

#include <roscpp_nodewrap/statistics/FrequencyStatistics.h>

#include <roscpp_nodewrap/diagnostics/CyclicEventTask.h>
#include <roscpp_nodewrap/diagnostics/FrequencyTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic task for frequency monitoring
    * 
    * This class provides the basis of all diagnostic tasks which monitor
    * the frequency of a cyclic event.
    */
  class FrequencyTask :
    public CyclicEventTask<FrequencyStatistics> {
  friend class DiagnosticTaskManager;
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
    
  protected:
    /** \brief ROS frequency task implementation
      * 
      * This class provides the private implementation of the frequency
      * task.
      */
    class Impl :
      public CyclicEventTask<FrequencyStatistics>::Impl {
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
