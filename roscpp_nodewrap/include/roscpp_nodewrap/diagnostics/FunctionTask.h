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

/** \file FunctionTask.h
  * \brief Header file providing the FunctionTask class interface
  */

#ifndef ROSCPP_NODEWRAP_FUNCTION_TASK_H
#define ROSCPP_NODEWRAP_FUNCTION_TASK_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/FunctionTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic function task
    * 
    * This class provides a diagnostic task which calls a function.
    */
  class FunctionTask :
    public DiagnosticTask {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Forward declaration of the function task options
      */
    typedef FunctionTaskOptions Options;
    
    /** \brief Default constructor
      */
    FunctionTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source function task which is being copied
      *   to this function task.
      */
    FunctionTask(const FunctionTask& src);
    
    /** \brief Destructor
      */
    ~FunctionTask();

  private:
    /** \brief ROS function task implementation
      * 
      * This class provides the private implementation of the function
      * task.
      */
    class Impl :
      public DiagnosticTask::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Fill out this function task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
      
      /** \brief The function called when the task should perform diagnostics
        */ 
      DiagnosticTaskCallback callback;
    };
  };
};

#endif
