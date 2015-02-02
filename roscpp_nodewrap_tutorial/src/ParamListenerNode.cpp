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

#include <ros/xmlrpc_manager.h>

#include "roscpp_nodewrap_tutorial/ParamListenerNode.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ParamListenerNode::ParamListenerNode() {
}

ParamListenerNode::~ParamListenerNode() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ParamListenerNode::init() {
  NODEWRAP_INFO("Hello!");
  
  NODEWRAP_INFO_STREAM("ParamListenerNode::init() " << __LINE__ << " " << ros::isInitialized());
  
  // This is problematic as it prevents cache parameters to be updated!!!
  ros::XMLRPCManager::instance()->unbind("paramUpdate");
  ros::XMLRPCManager::instance()->bind("paramUpdate",
    boost::bind(&ParamListenerNode::paramUpdate, this, _1, _2));
  
  bool bogus = false;
  getNodeHandle().setParam("bogus", bogus);
  getNodeHandle().getParamCached("bogus", bogus);
  
//   NODEWRAP_INFO_STREAM("ChatterNode::init() " << __LINE__);  
  
//   advertiseParam("chat/name", name, false);
//   advertiseParam("chat/initiate", initiate);
//   advertiseParam("chat/say", say, &ChatterNode::sayUpdate);
}

void ParamListenerNode::cleanup() {
  NODEWRAP_INFO("Good bye!");
}

void ParamListenerNode::paramUpdate(XmlRpc::XmlRpcValue& params,
    XmlRpc::XmlRpcValue& result) {
  NODEWRAP_INFO_STREAM("ParamListenerNode::paramUpdate() " << __LINE__);
  
//   result[0] = 1;
//   result[1] = std::string("");
//   result[2] = 0;
// 
//   ros::param::update((std::string)params[1], params[2]);
  
//   bool trigger = getNodeHandle().getParam("trigger", trigger);
//   NODEWRAP_INFO_STREAM("params[2] = " << params[2] << " trigger = " << trigger);
}

void ParamListenerNode::paramUpdate(const std::string& name) {
  NODEWRAP_INFO("Parameter value changed to %s!", name.c_str());
}

}
