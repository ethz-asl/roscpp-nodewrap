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
  *   (and its dependencies)
  */

#ifndef ROSCPP_NODEWRAP_FORWARDS_H
#define ROSCPP_NODEWRAP_FORWARDS_H

#include <ros/forwards.h>

#include <roscpp_nodewrap/diagnostics/DiagnosticsForwards.h>
#include <roscpp_nodewrap/timer/TimerForwards.h>
#include <roscpp_nodewrap/worker/WorkerForwards.h>

namespace roscpp_nodewrap_msgs {};
  
namespace nodewrap {
  using namespace roscpp_nodewrap_msgs;
  
  /** \brief Forward declaration of the node implementation
    */
  class NodeImpl;  
  /** \brief Forward declaration of the node implementation pointer type
    */
  typedef boost::shared_ptr<NodeImpl> NodeImplPtr;
  /** \brief Forward declaration of the node implementation weak pointer type
    */
  typedef boost::weak_ptr<NodeImpl> NodeImplWPtr;
};

#endif
