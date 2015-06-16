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

/** \file CompositeTask.h
  * \brief Header file providing the CompositeTask class interface
  */

#ifndef ROSCPP_NODEWRAP_COMPOSITE_TASK_H
#define ROSCPP_NODEWRAP_COMPOSITE_TASK_H

#include <list>

#include <roscpp_nodewrap/diagnostics/DiagnosticTaskOptions.h>
#include <roscpp_nodewrap/diagnostics/DiagnosticTaskManager.h>

namespace nodewrap {
  /** \brief Composite diagnostic task
    * 
    * This class provides a diagnostic task which is composed of other
    * diagnostic tasks.
    */
  class CompositeTask :
    public DiagnosticTask,
    public DiagnosticTaskManager {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Forward declaration of the composite task options
      */
    typedef DiagnosticTaskOptions Options;
    
    /** \brief Default constructor
      */
    CompositeTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source composite task which is being copied
      *   to this composite task.
      */
    CompositeTask(const CompositeTask& src);
    
    /** \brief Destructor
      */
    ~CompositeTask();

    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (DiagnosticTask::impl && DiagnosticTaskManager::impl) ?
        (void*)1 : (void*)0;
    };
    
    /** \brief Perform shutdown of the composite task
      */
    void shutdown();
    
  private:
    /** \brief ROS composite task implementation
      * 
      * This class provides the private implementation of the composite
      * task.
      */
    class Impl :
      public DiagnosticTask::Impl,
      public DiagnosticTaskManager::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Retrieve the node owning this composite task
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief Start the provided task (implementation)
        */
      void startTask(diagnostic_updater::DiagnosticTask& task);
      
      /** \brief Stop the referred task (implementation)
        */
      void stopTask(const std::string& name);
      
      /** \brief Fill out this composite task's status
        */
      void run(diagnostic_updater::DiagnosticStatusWrapper& status);
      
      /** \brief The diagnostic tasks by name
        */ 
      std::list<std::string> tasks;
      
      /** \brief The mutex guarding the tasks
        */ 
      boost::mutex taskMutex;
    };
    
    using DiagnosticTask::impl;
    
    /** \brief Reset the implemenation of this composite task
      */
    void resetImpl(Impl* impl);
  };
};

#endif
