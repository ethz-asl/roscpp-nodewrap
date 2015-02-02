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

/** \file ParamAdvertiserNode.h
  * \brief Header file providing the example ParamAdvertiserNode class interface
  */

#ifndef ROSCPP_NODEWRAP_TUTORIAL_PARAM_ADVERTISER_NODE_HPP
#define ROSCPP_NODEWRAP_TUTORIAL_PARAM_ADVERTISER_NODE_HPP

#include <roscpp_nodewrap/NodeImpl.h>

namespace nodewrap {
  /** \brief Example parameter advertiser node
    * 
    * This parameter advertiser node demonstrates the concept of using
    * parameter service server for advertising a parameter.
    */
  
  class ParamAdvertiserNode :
    public NodeImpl {
  public:
    /** \brief Default constructor
      * 
      * The constructor of the parameter advertiser node is left empty.
      */
    ParamAdvertiserNode();
    
    /** \brief Destructor
      * 
      * The destructor of the parameter advertiser node is left empty.
      */
    virtual ~ParamAdvertiserNode();
  
  protected:
    /** \brief Perform parameter advertiser node initialization
      * 
      * The initialization of the parameter advertiser node involves
      * advertising the targeted parameter.
      *
      * This is the initializer's implementation:
      * 
        \verbatim
        void ParamAdvertiserNode::init() {
        }
        \endverbatim
      * 
      * \see NodeImpl::init
      */
    void init();
    
    /** \brief Perform parameter advertiser node cleanup
      * 
      * Cleanup of the parameter advertiser node is just bogus. This is the
      * implementation:
      * 
        \verbatim
        void ParamAdvertiserNode::cleanup() {
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
