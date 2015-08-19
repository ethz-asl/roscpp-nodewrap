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

/** \file SubscriberImpl.h
  * \brief Header file providing the SubscriberImpl class interface
  */

#ifndef ROSCPP_NODEWRAP_SUBSCRIBER_IMPL_H
#define ROSCPP_NODEWRAP_SUBSCRIBER_IMPL_H

#include <ros/ros.h>

#include <boost/enable_shared_from_this.hpp>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Abstract ROS subscriber wrapper implementation
    * 
    * This class provides the abstract basis of the ROS subscriber
    * wrapper implementation.
    */
  class SubscriberImpl :
    public boost::enable_shared_from_this<SubscriberImpl> {
  friend class Subscriber;
  private:
    /** \brief Constructor
      */
    SubscriberImpl();
    
    /** \brief Destructor
      */
    virtual ~SubscriberImpl();
    
    /** \brief Initialize the subscriber implementation
      */
    virtual void init(const SubscriberOptions& defaultOptions) = 0;
          
    /** \brief Perform shutdown of the subscriber implementation
      */
    virtual void shutdown() = 0;
  };
};

#endif
