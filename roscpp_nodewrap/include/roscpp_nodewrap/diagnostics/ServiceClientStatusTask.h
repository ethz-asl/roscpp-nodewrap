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

/** \file ServiceClientStatusTask.h
  * \brief Header file providing the ServiceClientStatusTask class interface
  */

#ifndef ROSCPP_NODEWRAP_SERVICE_CLIENT_STATUS_TASK_H
#define ROSCPP_NODEWRAP_SERVICE_CLIENT_STATUS_TASK_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/ServiceClientStatusTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic service client status task
    * 
    * This class provides a diagnostic task which monitors the status
    * of a ROS service client.
    */
  class ServiceClientStatusTask :
    public DiagnosticTask {
  friend class DiagnosticTaskManager;
  friend class ServiceClient;
  public:
    /** \brief Forward declaration of the service client status task options
      */
    typedef ServiceClientStatusTaskOptions Options;
    
    /** \brief Default constructor
      */
    ServiceClientStatusTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source service client status task which is being
      *   copied to this service client status task.
      */
    ServiceClientStatusTask(const ServiceClientStatusTask& src);
    
    /** \brief Destructor
      */
    ~ServiceClientStatusTask();

    /** \brief Set the service client to be monitored by this task
      */
    void setServiceClient(const ServiceClient& serviceClient);
    
  private:
    /** \brief ROS service client status task implementation
      * 
      * This class provides the private implementation of the service client
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
      
      /** \brief Fill out this service client status task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
            
      /** \brief The service client monitored by this task
        */
      ServiceClientImplWPtr serviceClient;
    };
  };
};

#endif
