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

/** \file ServiceServerStatusTask.h
  * \brief Header file providing the ServiceServerStatusTask class interface
  */

#ifndef ROSCPP_NODEWRAP_SERVICE_SERVER_STATUS_TASK_H
#define ROSCPP_NODEWRAP_SERVICE_SERVER_STATUS_TASK_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/ServiceServerStatusTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic service server status task
    * 
    * This class provides a diagnostic task which monitors the status
    * of a ROS service server.
    */
  class ServiceServerStatusTask :
    public DiagnosticTask {
  friend class DiagnosticTaskManager;
  friend class ServiceServer;
  public:
    /** \brief Forward declaration of the service server status task options
      */
    typedef ServiceServerStatusTaskOptions Options;
    
    /** \brief Default constructor
      */
    ServiceServerStatusTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source service server status task which is being
      *   copied to this service server status task.
      */
    ServiceServerStatusTask(const ServiceServerStatusTask& src);
    
    /** \brief Destructor
      */
    ~ServiceServerStatusTask();

    /** \brief Set the service server to be monitored by this task
      */
    void setServiceServer(const ServiceServer& serviceServer);
    
  private:
    /** \brief ROS service server status task implementation
      * 
      * This class provides the private implementation of the service
      * server status task.
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
      
      /** \brief Fill out this service server status task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
            
      /** \brief The service server monitored by this task
        */
      ServiceServerImplWPtr serviceServer;
    };
  };
};

#endif
