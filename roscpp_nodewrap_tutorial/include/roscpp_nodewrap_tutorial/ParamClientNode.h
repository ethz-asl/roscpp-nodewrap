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
    * configuration and parameter service clients for accessing the
    * parameters advertised by another node.
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
    /** \brief The configuration service client of this node
      */
    ConfigClient configClient;
    
    /** \brief The parameter service client for the XML/RPC parameter
      */
    ParamClient xmlClient;
    
    /** \brief The parameter service client for the string parameter
      */
    ParamClient stringClient;
    
    /** \brief The parameter service client for the double parameter
      */
    ParamClient doubleClient;
    
    /** \brief The parameter service client for the integer parameter
      */
    ParamClient integerClient;
    
    /** \brief The parameter service client for the boolean parameter
      */
    ParamClient booleanClient;
    
    /** \brief Perform parameter client node initialization
      * 
      * The initialization of the parameter client node involves creating
      * a configuration service client (which connects to a configuration
      * service server) and then creating the different parameter service
      * clients through the interface methods of this configuration service
      * client. In constrast to using the node implementation's interface
      * methods for creating parameter service clients, the configuration
      * service client will thus identify the parameter services by querying
      * the connected configuration service server.
      *
      * Once the parameter service clients have been created, the initializer
      * attempts to retrieve the parameter values from the individually
      * connected parameter service servers and simply writes these values
      * to the console.
      * 
      * This is the initializer's implementation:
      * 
        \verbatim
        void ParamClientNode::init() {
          configClient = NodeImpl::configClient("config");
          
          xmlClient = configClient.paramClient<XmlRpc::XmlRpcValue>("xml");
          stringClient = configClient.paramClient<std::string>("string");
          doubleClient = configClient.paramClient<double>("double");
          integerClient = configClient.paramClient<int>("integer");
          booleanClient = configClient.paramClient<bool>("boolean");

          NODEWRAP_INFO("I'm sorry, did you just say you needed my brain?");
          
          NODEWRAP_INFO("Value of [xml]: %s",
            xmlClient.getParamValue<XmlRpc::XmlRpcValue>().toXml().c_str());
          NODEWRAP_INFO("Value of [string]: %s",
            stringClient.getParamValue<std::string>().c_str());
          NODEWRAP_INFO("Value of [double]: %lf",
            doubleClient.getParamValue<double>());
          NODEWRAP_INFO("Value of [integer]: %d",
            integerClient.getParamValue<int>());
          NODEWRAP_INFO("Value of [boolean]: %s",
            booleanClient.getParamValue<bool>() ? "true" : "false");
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
          NODEWRAP_INFO("So this is it. We're going to die.");
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
