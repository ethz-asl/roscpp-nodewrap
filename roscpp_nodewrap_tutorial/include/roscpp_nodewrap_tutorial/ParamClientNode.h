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

/** \file ParamClientNode.h
  * \brief Header file providing the example ParamClientNode class interface
  */

#ifndef ROSCPP_NODEWRAP_TUTORIAL_PARAM_CLIENT_NODE_HPP
#define ROSCPP_NODEWRAP_TUTORIAL_PARAM_CLIENT_NODE_HPP

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

namespace nodewrap {
  /** \brief Example parameter client node
    * 
    * This parameter client node demonstrates the concept of using
    * parameter service clients for subscribing to parameter updates.
    */
  class ParamClientNode :
    public NodeImpl {
  public:
    /** \brief Default constructor
      * 
      * The constructor of the parameter client node is left empty.
      */
    ParamClientNode();
    
    /** \brief Destructor
      * 
      * The destructor of the parameter client node is left empty.
      */
    virtual ~ParamClientNode();
    
  protected:
    /** \brief The parameter service client subscribing the XML/RPC parameter
      */
    ParamClient xmlClient;
    
    /** \brief Perform parameter client node initialization
      * 
      * The initialization of the parameter client node involves subscribing
      * to targeted parameter and registering a callback handler to be executed
      * in case of an update.
      *
      * This is the initializer's implementation:
      * 
        \verbatim
        void ParamClientNode::init() {
        }
        \endverbatim
      * 
      * \see NodeImpl::init
      */
    void init();
    
    /** \brief Perform parameter client node cleanup
      * 
      * Cleanup of the parameter client node is just bogus. This is the
      * implementation:
      * 
        \verbatim
        void ParamClientNode::cleanup() {
          NODEWRAP_INFO("Good bye!");
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
