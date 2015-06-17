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

/** \file WorkerStatusTask.h
  * \brief Header file providing the WorkerStatusTask class interface
  */

#ifndef ROSCPP_NODEWRAP_WORKER_STATUS_TASK_H
#define ROSCPP_NODEWRAP_WORKER_STATUS_TASK_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/WorkerStatusTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic worker status task
    * 
    * This class provides a diagnostic task which monitors the status
    * of a ROS worker.
    */
  class WorkerStatusTask :
    public DiagnosticTask {
  friend class DiagnosticTaskManager;
  friend class Worker;
  public:
    /** \brief Forward declaration of the worker status task options
      */
    typedef WorkerStatusTaskOptions Options;
    
    /** \brief Default constructor
      */
    WorkerStatusTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source worker status task which is being
      *   copied to this worker status task.
      */
    WorkerStatusTask(const WorkerStatusTask& src);
    
    /** \brief Destructor
      */
    ~WorkerStatusTask();

    /** \brief Set the worker to be monitored by this task
      */
    void setWorker(const Worker& worker);
    
  private:
    /** \brief ROS worker status task implementation
      * 
      * This class provides the private implementation of the worker
      * status task.
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
      
      /** \brief Fill out this worker status task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
            
      /** \brief The worker monitored by this task
        */
      WorkerImplWPtr worker;
    };
  };
};

#endif
