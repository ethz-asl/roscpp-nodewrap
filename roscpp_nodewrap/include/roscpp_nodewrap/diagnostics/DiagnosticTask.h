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

#include <boost/thread/mutex.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Abstract basis of the ROS diagnostic task
    * 
    * This class defines the abstract basis of all diagnostic tasks
    * of a ROS node(let).
    */
  class DiagnosticTask {
  friend class DiagnosticUpdater;
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
    
    /** \brief Perform shutdown of the diagnostic task
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const DiagnosticTask& task) const {
      return (impl < task.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const DiagnosticTask& task) const {
      return (impl == task.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const DiagnosticTask& task) const {
      return (impl != task.impl);
    };
    
  protected:
    /** \brief Abstract basis of the ROS diagnostic task implementation
      * 
      * This class provides the protected, abstract basis of the diagnostic
      * task implementation.
      */
    class Impl :
      public diagnostic_updater::DiagnosticTask {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief True, if this diagnostic task implementation is valid
        */
      bool isValid() const;
      
      /** \brief Remove the diagnostic task
        */
      void remove();
            
      /** \brief True, if this diagnostic task is valid
        */ 
      bool valid;
      
      /** \brief The mutex guarding this diagnostic task
        */ 
      mutable boost::mutex mutex;
        
      /** \brief The node implementation owning this diagnostic task
        */ 
      NodeImplPtr nodeImpl;
    };

    /** \brief Declaration of the diagnostic task implementation pointer
      *   type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the diagnostic task implementation weak
      *   pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The diagnostic task's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (protected version)
      */
    DiagnosticTask(const ImplPtr& impl);
  };
};

#endif
