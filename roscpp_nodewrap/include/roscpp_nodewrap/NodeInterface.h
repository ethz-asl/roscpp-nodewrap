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

/** \file NodeInterface.h
  * \brief Header file providing the NodeInterface class interface
  */

#ifndef ROSCPP_NODEWRAP_NODE_INTERFACE_H
#define ROSCPP_NODEWRAP_NODE_INTERFACE_H

#include <ros/ros.h>

namespace nodewrap {
  /** \brief Abstract interface of a ROS node(let).
    * 
    * This class defines the abstract, minimal interface of all node wrappers
    * and is implemented by NodeImpl.
    */

  class NodeInterface {
  public:
    /** \brief Default constructor
      */
    NodeInterface();
    
    /** \brief Destructor
      */
    virtual ~NodeInterface();
    
    /** \brief Retrieve the wrapped node(let)'s name
      * 
      * \return The name of the wrapped node(let).
      * 
      * \note This function must be implemented by the subclass.
      */
    virtual const std::string& getName() const = 0;
    
    /** \brief Retrieve the wrapped node(let)'s ROS node handle
      * 
      * By convention, the ROS node handle delivered by the implementation
      * of this accessor should refer to the private ROS node handle, i.e.,
      * the ROS node handle with this node(let)'s custom namespace and
      * remappings.
      * 
      * \return The private ROS node handle used by the wrapped node(let).
      * 
      * \note This function must be implemented by the subclass.
      */
    virtual ros::NodeHandle& getNodeHandle() const = 0;
    
    /** \brief Query if the wrapped node(let) is a ROS nodelet
      * 
      * \return True if the wrapped node(let) is a ROS nodelet.
      * 
      * \note This function must be implemented by the subclass.
      */
    virtual bool isNodelet() const = 0;
  };
};

#endif
