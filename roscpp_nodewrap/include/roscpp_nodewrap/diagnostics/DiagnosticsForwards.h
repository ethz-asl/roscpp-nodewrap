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

/** \file DiagnosticsForwards.h
  * \brief Header file providing forward declarations for the node diagnostics
  */

#ifndef ROSCPP_NODEWRAP_DIAGNOSTICS_FORWARDS_H
#define ROSCPP_NODEWRAP_DIAGNOSTICS_FORWARDS_H

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>

namespace nodewrap {
  /** \brief Forward declaration of the diagnostic task
    */
  class DiagnosticTask;
  /** \brief Forward declaration of the diagnostic task pointer type
    */
  typedef boost::shared_ptr<DiagnosticTask> DiagnosticTaskPtr;
  /** \brief Forward declaration of the diagnostic task weak pointer type
    */
  typedef boost::weak_ptr<DiagnosticTask> DiagnosticTaskWPtr;
  
  /** \brief Forward declaration of the diagnostic task manager
    */
  class DiagnosticTaskManager;
  /** \brief Forward declaration of the diagnostic task manager pointer type
    */
  typedef boost::shared_ptr<DiagnosticTaskManager> DiagnosticTaskManagerPtr;
  /** \brief Forward declaration of the diagnostic task manager weak pointer
    *   type
    */
  typedef boost::weak_ptr<DiagnosticTaskManager> DiagnosticTaskManagerWPtr;
  
  /** \brief Forward declaration of the diagnostic updater
    */
  class DiagnosticUpdater;
  /** \brief Forward declaration of the diagnostic updater pointer type
    */
  typedef boost::shared_ptr<DiagnosticUpdater> DiagnosticUpdaterPtr;
  /** \brief Forward declaration of the diagnostic updater weak pointer type
    */
  typedef boost::weak_ptr<DiagnosticUpdater> DiagnosticUpdaterWPtr;
  
  /** \brief Forward declaration of the diagnostic task callback function type
    */
  typedef diagnostic_updater::TaskFunction DiagnosticTaskCallback;
};

#endif
