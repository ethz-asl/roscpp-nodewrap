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

/** \file PublisherStatusTask.h
  * \brief Header file providing the PublisherStatusTask class interface
  */

#ifndef ROSCPP_NODEWRAP_PUBLISHER_STATUS_TASK_H
#define ROSCPP_NODEWRAP_PUBLISHER_STATUS_TASK_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/PublisherStatusTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic publisher status task
    * 
    * This class provides a diagnostic task which monitors the status
    * of a ROS publisher.
    */
  class PublisherStatusTask :
    public DiagnosticTask {
  friend class DiagnosticTaskManager;
  friend class Publisher;
  public:
    /** \brief Forward declaration of the publisher status task options
      */
    typedef PublisherStatusTaskOptions Options;
    
    /** \brief Default constructor
      */
    PublisherStatusTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source publisher status task which is being
      *   copied to this publisher status task.
      */
    PublisherStatusTask(const PublisherStatusTask& src);
    
    /** \brief Destructor
      */
    ~PublisherStatusTask();

    /** \brief Set the publisher to be monitored by this task
      */
    void setPublisher(const Publisher& publisher);
    
  private:
    /** \brief ROS publisher status task implementation
      * 
      * This class provides the private implementation of the publisher
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
      
      /** \brief Fill out this publisher status task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
            
      /** \brief The publisher monitored by this task
        */
      PublisherImplWPtr publisher;
      
      /** \brief The minimum number of subscribers of the publisher
        *   diagnosed by this task
        */
      size_t minNumSubscribers;
    };
  };
};

#endif
