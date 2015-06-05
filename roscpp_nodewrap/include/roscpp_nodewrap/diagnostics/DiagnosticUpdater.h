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

/** \file DiagnosticUpdater.h
  * \brief Header file providing the DiagnosticUpdater class interface
  */

#ifndef ROSCPP_NODEWRAP_DIAGNOSTIC_UPDATER_H
#define ROSCPP_NODEWRAP_DIAGNOSTIC_UPDATER_H

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <roscpp_nodewrap/Forwards.h>

#include <roscpp_nodewrap/diagnostics/DiagnosticTask.h>

namespace nodewrap {
  /** \brief ROS diagnostic updater
    * 
    * This class provides the diagnostic updater of a ROS node(let).
    */
  class DiagnosticUpdater {
  friend class NodeImpl;
  friend class DiagnosticTask;
  public:
    /** \brief Default constructor
      */
    DiagnosticUpdater();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source diagnostic updater which is being
      *   copied to this diagnostic updater.
      */
    DiagnosticUpdater(const DiagnosticUpdater& src);
    
    /** \brief Destructor
      */
    ~DiagnosticUpdater();
    
    /** \brief Set the hardware identifier to be announced by this
      *   diagnostic updater
      * 
      * \param[in] hardwareId The hardware identifier to be announced.
      */
    void setHardwareId(const std::string& hardwareId);
    
    /** \brief Perform shutdown of the diagnostic updater
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return impl ? (void*)1 : (void*)0;
    };
    
  private:
    /** \brief ROS diagnostic updater implementation
      * 
      * This class provides the private implementation of the diagnostic
      * updater.
      */
    class Impl :
      public diagnostic_updater::Updater {
    public:
      /** \brief Default constructor
        */
      Impl(const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Set the hardware identifier to be announced by this
        *   diagnostic updater (implementation)
        */
      void setHardwareId(const std::string& hardwareId);
      
      /** \brief Perform shutdown of the diagnostic updater (implementation)
        */
      void shutdown();
            
      /** \brief The timer callback of this diagnostic updater
        */ 
      void timerCallback(const ros::TimerEvent& timerEvent);
      
      /** \brief The timer controlling this diagnostic updater
        */ 
      ros::Timer timer;
      
      /** \brief The node implementation owning this diagnostic updater
        */ 
      NodeImplPtr nodeImpl;
      
      /** \brief The diagnostic updater's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The diagnostic tasks run by this diagnostics updater
        */ 
      std::map<std::string, DiagnosticTask::ImplWPtr> tasks;
    };
    
    /** \brief Declaration of the diagnostic updater implementation
      *   pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the diagnostic updater implementation
      *   weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The diagnostic updater's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    DiagnosticUpdater(const NodeImplPtr& nodeImpl);
    
    /** \brief Add a diagnostic task to this diagnostic updater
      */
    template <class T> T addTask(const std::string& name, const typename
      T::Options& defaultOptions);
  };
};

#include <roscpp_nodewrap/diagnostics/DiagnosticUpdater.tpp>

#endif
