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

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class C> Node<C>::Node(const std::string& ns, const ros::M_string&
    remappings) :
  privateNodeHandle(new ros::NodeHandle(ns, remappings)) {
  this->init();
}
  
template <class C> Node<C>::Node(const Node& parent, const std::string& ns) :
  privateNodeHandle(new ros::NodeHandle(parent.nodeHandle, ns)) {
  this->init();
}

template <class C> Node<C>::Node(const Node& parent, const std::string& ns,
    const ros::M_string& remappings) :
  privateNodeHandle(new ros::NodeHandle(parent.nodeHandle, ns, remappings)) {
  this->init();
}

template <class C> Node<C>::Node(const Node& src) :
  C(src),
  privateNodeHandle(src.privateNodeHandle) {
  this->init();
}
                                 
template <class C> Node<C>::~Node() {
}
  
/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class C> const std::string& Node<C>::getName() const {
  return ros::this_node::getName();
}

template <class C> bool Node<C>::isNodelet() const {
  return false;
}
  
template <class C> ros::NodeHandle& Node<C>::getNodeHandle() const {
  return *privateNodeHandle;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <class C> Node<C>& Node<C>::operator=(const Node<C>& src) {
  C::operator=(src);
  
  privateNodeHandle = src.privateNodeHandle;
  
  return *this;
}

}
