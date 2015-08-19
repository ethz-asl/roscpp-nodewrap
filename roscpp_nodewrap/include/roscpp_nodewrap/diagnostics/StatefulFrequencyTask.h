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

/** \file StatefulFrequencyTask.h
  * \brief Header file providing the StatefulFrequencyTask class interface
  */

#ifndef ROSCPP_NODEWRAP_STATEFUL_FREQUENCY_TASK_H
#define ROSCPP_NODEWRAP_STATEFUL_FREQUENCY_TASK_H

#include <roscpp_nodewrap/diagnostics/FrequencyTask.h>

namespace nodewrap {
  /** \brief Diagnostic task for stateful frequency monitoring
    * 
    * This class provides a stateful diagnostic task to monitor the
    * frequency of a cyclic event.
    */
  class StatefulFrequencyTask :
    public FrequencyTask {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Forward declaration of the stateful frequency task options
      */
    typedef FrequencyTask::Options Options;
    
    /** \brief Default constructor
      */
    StatefulFrequencyTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source stateful frequency task which is being
      *   copied to this stateful frequency task.
      */
    StatefulFrequencyTask(const StatefulFrequencyTask& src);
    
    /** \brief Destructor
      */
    ~StatefulFrequencyTask();
    
    /** \brief Enable this stateful frequency task
      */
    void enable();
    
    /** \brief Enable this stateful frequency task
      */
    void disable(bool clear = true);
    
  private:
    /** \brief ROS stateful frequency task implementation
      * 
      * This class provides the private implementation of the stateful
      * frequency task.
      */
    class Impl :
      public FrequencyTask::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Enable this stateful frequency task (implementation)
        */
      void enable();
      
      /** \brief Enable this stateful frequency task (implementation)
        */
      void disable(bool clear = true);
      
      /** \brief Update the statistics of this stateful frequency task
        */
      void update(const double& sample);
      
      /** \brief Fill out this stateful frequency task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
      
      /** \brief If true, this stateful frequency task has been enabled
        */
      bool enabled;
      
      /** \brief The mutex guarding the frequency task's state
        */ 
      mutable boost::mutex stateMutex;
    };
  };
};

#endif
