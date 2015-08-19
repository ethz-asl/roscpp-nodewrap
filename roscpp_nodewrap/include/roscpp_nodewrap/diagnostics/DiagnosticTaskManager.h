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

/** \file DiagnosticTaskManager.h
  * \brief Header file providing the DiagnosticTaskManager class interface
  */

#ifndef ROSCPP_NODEWRAP_DIAGNOSTIC_TASK_MANAGER_H
#define ROSCPP_NODEWRAP_DIAGNOSTIC_TASK_MANAGER_H

#include <roscpp_nodewrap/Manager.h>

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>

namespace nodewrap {
  /** \brief Abstract basis of the ROS diagnostic task manager
    * 
    * This class provides the abstract basis of a diagnostic task manager
    * for use with the ROS node implementation.
    */
  class DiagnosticTaskManager :
    public Manager<DiagnosticTask, std::string> {
  friend class DiagnosticTask;
  public:
    /** \brief Default constructor
      */
    DiagnosticTaskManager();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source diagnostic task manager which is being
      *   copied to this diagnostic task manager.
      */
    DiagnosticTaskManager(const DiagnosticTaskManager& src);
    
    /** \brief Destructor
      */
    ~DiagnosticTaskManager();
    
    /** \brief Add a diagnostic task to this diagnostic task manager
      */
    template <class T> T addTask(const std::string& name, const typename
      T::Options& defaultOptions = typename T::Options());
    
  protected:
    /** \brief Abstract basis of the ROS diagnostic task manager
      *   implementation
      * 
      * This class provides the protected, abstract basis of the diagnostic
      * task manager implementation.
      */
    class Impl :
      public Manager<DiagnosticTask, std::string>::Impl {
    public:
      /** \brief Default constructor
        */
      Impl();
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Start the provided task
        */
      virtual void startTask(diagnostic_updater::DiagnosticTask& task) = 0;
      
      /** \brief Stop the referred task
        */
      virtual void stopTask(const std::string& name) = 0;
      
      /** \brief Perform shutdown of the diagnostic task manager
        *   (implementation)
        */
      void shutdown();
    };
  };
};

#include <roscpp_nodewrap/diagnostics/DiagnosticTaskManager.tpp>

#endif
