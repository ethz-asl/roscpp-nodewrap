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

/** \file SubscriberStatusTask.h
  * \brief Header file providing the SubscriberStatusTask class interface
  */

#ifndef ROSCPP_NODEWRAP_SUBSCRIBER_STATUS_TASK_H
#define ROSCPP_NODEWRAP_SUBSCRIBER_STATUS_TASK_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>
#include <roscpp_nodewrap/diagnostics/SubscriberStatusTaskOptions.h>

namespace nodewrap {
  /** \brief Diagnostic subscriber status task
    * 
    * This class provides a diagnostic task which monitors the status
    * of a ROS subscriber.
    */
  class SubscriberStatusTask :
    public DiagnosticTask {
  friend class DiagnosticTaskManager;
  friend class Subscriber;
  public:
    /** \brief Forward declaration of the subscriber status task options
      */
    typedef SubscriberStatusTaskOptions Options;
    
    /** \brief Default constructor
      */
    SubscriberStatusTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source subscriber status task which is being
      *   copied to this subscriber status task.
      */
    SubscriberStatusTask(const SubscriberStatusTask& src);
    
    /** \brief Destructor
      */
    ~SubscriberStatusTask();

    /** \brief Set the subscriber to be monitored by this task
      */
    void setSubscriber(const Subscriber& subscriber);
    
  private:
    /** \brief ROS subscriber status task implementation
      * 
      * This class provides the private implementation of the subscriber
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
      
      /** \brief Fill out this subscriber status task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
            
      /** \brief The subscriber monitored by this task
        */
      SubscriberImplWPtr subscriber;
      
      /** \brief The minimum number of publisher of the subscriber
        *   diagnosed by this task
        */
      size_t minNumPublishers;
    };
  };
};

#endif
