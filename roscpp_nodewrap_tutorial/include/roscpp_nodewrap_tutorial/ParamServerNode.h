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

/** \file ParamServerNode.h
  * \brief Header file providing the example ParamServerNode class interface
  */

#ifndef ROSCPP_NODEWRAP_TUTORIAL_PARAM_SERVER_NODE_HPP
#define ROSCPP_NODEWRAP_TUTORIAL_PARAM_SERVER_NODE_HPP

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

namespace nodewrap {
  /** \brief Example parameter server node
    * 
    * This parameter server node demonstrates the concept of using
    * parameter service server for advertising a parameter.
    */
  class ParamServerNode :
    public NodeImpl {
  public:
    /** \brief Default constructor
      * 
      * The constructor of the parameter server node is left empty.
      */
    ParamServerNode();
    
    /** \brief Destructor
      * 
      * The destructor of the parameter server node is left empty.
      */
    virtual ~ParamServerNode();
  
  protected:
    /** \brief The parameter service server advertising the XML/RPC parameter
      */
    ParamServer xmlServer;
    
    /** \brief The parameter service server advertising the string parameter
      */
    ParamServer stringServer;
    
    /** \brief The parameter service server advertising the double parameter
      */
    ParamServer doubleServer;
    
    /** \brief The parameter service server advertising the integer parameter
      */
    ParamServer integerServer;
    
    /** \brief The parameter service server advertising the boolean parameter
      */
    ParamServer booleanServer;
    
    /** \brief Perform parameter server node initialization
      * 
      * The initialization of the parameter server node involves advertising
      * the targeted parameter.
      *
      * This is the initializer's implementation:
      * 
        \verbatim
        void ParamServerNode::init() {
          NODEWRAP_INFO("I think you ought to know I'm feeling very depressed.");
        }
        \endverbatim
      * 
      * \see NodeImpl::init
      */
    void init();
    
    /** \brief Perform parameter server node cleanup
      * 
      * Cleanup of the parameter server node is just bogus. This is the
      * implementation:
      * 
        \verbatim
        void ParamServerNode::cleanup() {
          NODEWRAP_INFO("I told you this would all end in tears.");
        }
        \endverbatim
      * 
      * 
      * \see NodeImpl::cleanup
      */
    void cleanup();
  };
};

#endif
