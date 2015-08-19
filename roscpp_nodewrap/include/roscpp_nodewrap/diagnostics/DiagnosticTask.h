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

/** \file DiagnosticTask.h
  * \brief Header file providing the DiagnosticTask class interface
  */

#ifndef ROSCPP_NODEWRAP_DIAGNOSTIC_TASK_H
#define ROSCPP_NODEWRAP_DIAGNOSTIC_TASK_H

#include <ros/ros.h>

// #include <diagnostic_updater/diagnostic_updater.h>

#include <roscpp_nodewrap/Managed.h>

namespace nodewrap {
  /** \brief Abstract basis of the ROS diagnostic task
    * 
    * This class defines the abstract basis of all diagnostic tasks
    * of a ROS node(let).
    */
  class DiagnosticTask :
    public Managed<DiagnosticTask, std::string> {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Default constructor
      */
    DiagnosticTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source diagnostic task which is being copied
      *   to this diagnostic task.
      */
    DiagnosticTask(const DiagnosticTask& src);
    
    /** \brief Destructor
      */
    ~DiagnosticTask();
    
    /** \brief Retrieve this diagnostic task's name
      */
    std::string getName() const;
    
    /** \brief Start this diagnostic task
      */
    void start();
          
    /** \brief Stop this diagnostic task
      */
    void stop();
            
  protected:
    /** \brief Abstract basis of the ROS diagnostic task implementation
      * 
      * This class provides the protected, abstract basis of the diagnostic
      * task implementation.
      */
    class Impl :
      public Managed<DiagnosticTask, std::string>::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Returns true, as the diagnostic task implementation is
        *   always valid
        */
      bool isValid() const;
      
      /** \brief Start this diagnostic task (implementation)
        */
      void start();
            
      /** \brief Fill out this diagnostic task's status
        */
      virtual void run(diagnostic_updater::DiagnosticStatusWrapper&
        status) = 0;
      
      /** \brief Stop this diagnostic task (implementation)
        */
      void stop();
            
      /** \brief Perform shutdown of this diagnostic task (implementation)
        */
      void shutdown();
      
      /** The ROS diagnostic task
        */ 
      diagnostic_updater::FunctionDiagnosticTask task;
      
      /** \brief True, if this diagnostic task has been removed
        */ 
      bool started;
    };
  
  private:
    /** \brief Reset the implemenation of this diagnostic task
      */
    void resetImpl(Impl* impl);
  };
};

#endif
