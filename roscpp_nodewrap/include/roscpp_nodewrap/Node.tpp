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

#include <roscpp_nodewrap/NodeImpl.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class C> Node<C>::Node(const std::string& ns, const ros::M_string&
    remappings) :
  impl(new C()) {
  impl->start(ros::this_node::getName(), false, ros::NodeHandlePtr(
    new ros::NodeHandle(ns, remappings)));
}
  
template <class C> Node<C>::Node(const Node& parent, const std::string& ns) :
  impl(new C()) {
  impl->start(ros::this_node::getName(), false, ros::NodeHandlePtr(
    new ros::NodeHandle(parent.nodeHandle, ns)));
}

template <class C> Node<C>::Node(const Node& parent, const std::string& ns,
    const ros::M_string& remappings) :
  impl(new C()) {
  impl->start(ros::this_node::getName(), false, ros::NodeHandlePtr(
    new ros::NodeHandle(parent.nodeHandle, ns, remappings)));
}

template <class C> Node<C>::Node(const Node& src) :
  impl(src.impl) {
}
                                 
template <class C> Node<C>::~Node() {
  if (impl.unique() && impl->isValid())
    impl->shutdown();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <class C> Node<C>& Node<C>::operator=(const Node<C>& src) {
  if (impl.unique() && impl->isValid())
    impl->shutdown();
  
  impl = src.impl;
  
  return *this;
}

}
