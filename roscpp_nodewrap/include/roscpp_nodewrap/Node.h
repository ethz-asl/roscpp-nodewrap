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

/** \file Node.h
  * \brief Header file providing the Node class interface
  */

#ifndef ROSCPP_NODEWRAP_NODE_H
#define ROSCPP_NODEWRAP_NODE_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS node template wrapper
    * 
    * This class is a templated wrapper for native ROS nodes. Its sole
    * template parameter is expected to subclass the NodeImpl interface.
    */
  
  template <class C> class Node {
  public:
    /** \brief Default constructor
      * 
      * \param[in] ns The namespace for the node handle of this node.
      *   This acts in addition to any namespace assigned to this node,
      *   e.g., if the node's namespace is "/a" and the namespace passed
      *   in here is "b", all topics, services, and parameters will be
      *   prefixed with "/a/b/".
      * \param[in] remappings The remappings for the ROS node handle of
      *   this node.
      * 
      * \note In contrast to the default constructor of the ROS node handle,
      *   the default namespace of this node is the private namespace.
      * 
      * \see ros::NodeHandle::NodeHandle
      */
    Node(const std::string& ns = std::string("~"), const ros::M_string&
      remappings = ros::M_string());
    
    /** \brief Parent constructor
      * 
      * \param[in] parent The parent node of this node. Essentially, the
      *   ROS node handle of this node will be constructed by passing the
      *   ROS node handle of the specified parent node.
      * \param[in] ns The namespace for the node handle of this node.
      *   This acts in addition to any namespace assigned to the parent
      *   node handle, e.g., if the parent node handle's namespace is "/a"
      *   and the namespace passed in here is "b", all topics, services,
      *   and parameters will be prefixed with "/a/b/".
      * 
      * \see ros::NodeHandle::NodeHandle
      */
    Node(const Node& parent, const std::string& ns);
    
    /** \brief Parent constructor
      * 
      * This constructor is an overloaded constructor, provided for
      * convenience. In addition to the parameters accepted by the above
      * version of the constructor, it takes the following parameter:
      * 
      * \param[in] remappings The remappings for the ROS node handle of
      *   this node.
      * 
      * \see ros::NodeHandle::NodeHandle
      */
    Node(const Node& parent, const std::string& ns, const ros::M_string&
      remappings = ros::M_string());
    
    /** \brief Copy constructor
      * 
      * When a node is copied, its ROS node handle inherits the namespace of
      * the ROS node handle associated with the node being copied, and
      * increments the reference count of the global ROS node state by 1.
      * 
      * \param[in] src The source node which is being copied to this node.
      * 
      * \see ros::NodeHandle::NodeHandle
      */
    Node(const Node& src);
    
    /** \brief Destructor
      */
    virtual ~Node();
    
    /** \brief Assignment operator
      * 
      * \param[in] src The source node which is being copied to this node.
      * \return A non-const reference to this node.
      * 
      * \see ros::NodeHandle::operator=
      */
    Node& operator=(const Node& src);

  private:
    /** \brief The node's implementation
      */
    NodeImplPtr impl;
  };
};

#include <roscpp_nodewrap/Node.tpp>

#endif
