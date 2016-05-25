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

/** \file WorkerImpl.h
  * \brief Header file providing the WorkerImpl class interface
  */

#ifndef ROSCPP_NODEWRAP_WORKER_IMPL_H
#define ROSCPP_NODEWRAP_WORKER_IMPL_H

#include <ros/ros.h>

#include <boost/enable_shared_from_this.hpp>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Abstract ROS worker implementation
    * 
    * This class provides the abstract basis of the ROS worker implementation.
    */
  class WorkerImpl :
    public boost::enable_shared_from_this<WorkerImpl> {
  friend class Worker;
  friend class WorkerEvent;
  private:
    /** \brief Constructor
      */
    WorkerImpl();
    
    /** \brief Destructor
      */
    virtual ~WorkerImpl();
    
    /** \brief True, if this worker has been canceled (implementation)
      */
    virtual bool isCanceled() const = 0;
    
    /** \brief Initialize the worker implementation
      */
    virtual void init(const WorkerOptions& defaultOptions) = 0;
          
    /** \brief Perform shutdown of the worker implementation
      */
    virtual void shutdown() = 0;
  };
};

#endif
