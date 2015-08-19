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

/** \file PublisherImpl.h
  * \brief Header file providing the PublisherImpl class interface
  */

#ifndef ROSCPP_NODEWRAP_PUBLISHER_IMPL_H
#define ROSCPP_NODEWRAP_PUBLISHER_IMPL_H

#include <ros/ros.h>

#include <boost/enable_shared_from_this.hpp>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Abstract ROS publisher wrapper implementation
    * 
    * This class provides the abstract basis of the ROS publisher
    * wrapper implementation.
    */
  class PublisherImpl :
    public boost::enable_shared_from_this<PublisherImpl> {
  friend class Publisher;
  private:
    /** \brief Constructor
      */
    PublisherImpl();
    
    /** \brief Destructor
      */
    virtual ~PublisherImpl();
    
    /** \brief Initialize the publisher implementation
      */
    virtual void init(const PublisherOptions& defaultOptions) = 0;
          
    /** \brief Perform shutdown of the publisher implementation
      */
    virtual void shutdown() = 0;
  };
};

#endif
