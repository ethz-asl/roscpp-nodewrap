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

/** \file Forwards.h
  * \brief Header file providing forward declarations for the node wrapper
  */

#ifndef ROSCPP_NODEWRAP_FORWARDS_H
#define ROSCPP_NODEWRAP_FORWARDS_H

#include <ros/ros.h>

namespace roscpp_nodewrap {};
  
namespace nodewrap {
  using namespace roscpp_nodewrap;
  
  /** \brief Forward declaration of the node implementation
    */
  class NodeImpl;  
  /** \brief Forward declaration of the node implementation pointer type
    */
  typedef boost::shared_ptr<NodeImpl> NodeImplPtr;
  /** \brief Forward declaration of the node implementation weak pointer type
    */
  typedef boost::weak_ptr<NodeImpl> NodeImplWPtr;
  
  /** \brief Forward declaration of the worker
    */
  class Worker;
  /** \brief Forward declaration of the worker pointer type
    */
  typedef boost::shared_ptr<Worker> WorkerPtr;
  /** \brief Forward declaration of the worker weak pointer type
    */
  typedef boost::weak_ptr<Worker> WorkerWPtr;
  
  /** \brief Forward declaration of the worker manager
    */
  class WorkerManager;
  /** \brief Forward declaration of the worker manager pointer type
    */
  typedef boost::shared_ptr<WorkerManager> WorkerManagerPtr;
  /** \brief Forward declaration of the worker manager weak pointer type
    */
  typedef boost::weak_ptr<WorkerManager> WorkerManagerWPtr;
  
  /** \brief Forward declaration of the worker queue callback
    */
  class WorkerQueueCallback;
  
  /** \brief Forward declaration of the worker queue callback function
    *   type
    */
  typedef boost::function<void()> WorkerQueueCallbackCallback;
  
  /** \brief Forward declaration of the worker event
    */
  class WorkerEvent;
  
  /** \brief Forward declaration of the worker callback function type
    */
  typedef boost::function<bool(const WorkerEvent&)> WorkerCallback;
};

#endif
