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

#include <roscpp_nodewrap/diagnostics/DiagnosticTaskManager.h>

namespace nodewrap {
  /** \brief ROS diagnostic updater
    * 
    * This class provides the diagnostic updater of a ROS node(let).
    */
  class DiagnosticUpdater :
    public DiagnosticTaskManager {
  friend class NodeImpl;
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
    
  private:
    /** \brief ROS diagnostic updater implementation
      * 
      * This class provides the private implementation of the diagnostic
      * updater.
      */
    class Impl :
      public DiagnosticTaskManager::Impl {
    public:
      /** \brief Default constructor
        */
      Impl(const NodeImplPtr& node);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Retrieve the node owning this diagnostic updater
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief Set the hardware identifier to be announced by this
        *   diagnostic updater (implementation)
        */
      void setHardwareId(const std::string& hardwareId);
      
      /** \brief Start the provided task (implementation)
        */
      void startTask(diagnostic_updater::DiagnosticTask& task);
      
      /** \brief Stop the referred task (implementation)
        */
      void stopTask(const std::string& name);
      
      /** \brief The timer callback of this diagnostic updater
        */ 
      void timerCallback(const ros::TimerEvent& timerEvent);
      
      /** \brief The ROS diagnostic updater
        */ 
      diagnostic_updater::Updater updater;
      
      /** \brief The timer controlling this diagnostic updater
        */ 
      Timer timer;
      
      /** \brief The node implementation owning this diagnostic updater
        */ 
      NodeImplPtr node;
    };

    /** \brief Constructor (private version)
      */
    DiagnosticUpdater(const NodeImplPtr& node);
  };
};

#endif
