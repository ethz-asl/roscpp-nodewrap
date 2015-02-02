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

/** \file ParamListenerNode.h
  * \brief Header file providing the example ParamListenerNode class interface
  */

#ifndef ROSCPP_NODEWRAP_TUTORIAL_PARAMLISTENERNODE_HPP
#define ROSCPP_NODEWRAP_TUTORIAL_PARAMLISTENERNODE_HPP

#include <roscpp_nodewrap/NodeImpl.h>

namespace nodewrap {
  /** \brief Example parameter listener node
    * 
    * This parameter listener node demonstrates the concept of using
    * parameter service clients for listening to parameter updates.
    */
  
  class ParamListenerNode :
    public NodeImpl {
  public:
    /** \brief Default constructor
      * 
      * The constructor of the parameter listener node is left empty.
      */
    ParamListenerNode();
    
    /** \brief Destructor
      * 
      * The destructor of the parameter listener node is left empty.
      */
    virtual ~ParamListenerNode();
    
  protected:
    /** \brief Perform parameter listener node initialization
      * 
      * The initialization of the parameter listener node involves subscribing
      * to changes of the targeted parameter and registering a callback handler
      * to be executed in case of an update.
      *
      * This is the initializer's implementation:
      * 
        \verbatim
        void ParamListenerNode::init() {
        }
        \endverbatim
      * 
      * \see NodeImpl::init
      */
    void init();
    
    /** \brief Perform parameter listener node cleanup
      * 
      * Cleanup of the parameter listener node is just bogus. This is the
      * implementation:
      * 
        \verbatim
        void ParamListenerNode::cleanup() {
          NODEWRAP_INFO("Good bye!");
        }
        \endverbatim
      * 
      * 
      * \see NodeImpl::cleanup
      */
    void cleanup();
        
    void paramUpdate(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
    
    void paramUpdate(const std::string& name);
  };
};

#endif
