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

/** \file Nodelet.h
  * \brief Header file providing the Nodelet class interface
  */

#ifndef ROSCPP_NODEWRAP_NODELET_H
#define ROSCPP_NODEWRAP_NODELET_H

#include <nodelet/nodelet.h>

namespace nodewrap {
  /** \brief ROS nodelet template wrapper
    * 
    * This class is a templated wrapper for native ROS nodelets. Its sole
    * template parameter usually subclasses the NodeImpl interface.
    */
  
  template <class C> class Nodelet :
    public C,
    public nodelet::Nodelet {
  public:
    /** \brief Default constructor
      * 
      * \note No constructor overloading is provided to allow for
      *   construction when dynamically loaded.
      * 
      * \see nodelet::Nodelet::Nodelet
      */
    Nodelet();
    
    /** \brief Destructor
      */
    virtual ~Nodelet();
    
    /** \brief Retrieve the nodelet's name
      * 
      * \return The nodelet's name.
      * 
      * \see NodeInterface::getName
      */
    const std::string& getName() const;
    
    /** \brief Query if the nodelet is a ROS nodelet
      * 
      * \return In this implementation, the method always returns true.
      * 
      * \see NodeInterface::isNodelet
      */
    bool isNodelet() const;
    
    /** \brief Virtual override of the ROS nodelet's initialization
      * 
      * This method essentially calls C::init() to delegate the initialization
      * to the node implementation.
      * 
      * \see nodelet::Nodelet::onInit
      */
    void onInit();
  protected:
    /** \brief Retrieve the node's ROS node handle
      * 
      * \return The private ROS node handle with the multi-threaded callback
      *   queue of the nodelet manager.
      * 
      * \see NodeInterface::getNodeHandle
      */
    ros::NodeHandle& getNodeHandle() const;
  };
};

#include <roscpp_nodewrap/Nodelet.tpp>

#include <roscpp_nodewrap/Pluginlib.h>

#endif
