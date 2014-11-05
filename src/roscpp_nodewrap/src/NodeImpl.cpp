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

#include "roscpp_nodewrap/NodeImpl.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

NodeImpl::NodeImpl() {
}

NodeImpl::~NodeImpl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

ros::Publisher NodeImpl::advertise(const std::string& param, const
    ros::AdvertiseOptions& defaultOptions) {
  std::string ns = std::string("publishers/")+param;  
  ros::AdvertiseOptions options = defaultOptions;
  options.topic = getParam(ns+"/topic", defaultOptions.topic);
  options.queue_size = getParam(ns+"/queue_size",
    (int)defaultOptions.queue_size);
  options.latch = getParam(ns+"/latch", defaultOptions.latch);
  
  return this->getNodeHandle().advertise(options);
}

ros::Subscriber NodeImpl::subscribe(const std::string& param, const
    ros::SubscribeOptions& defaultOptions) {
  std::string ns = std::string("subscribers/")+param;  
  ros::SubscribeOptions options = defaultOptions;
  options.topic = getParam(ns+"/topic", defaultOptions.topic);
  options.queue_size = getParam(ns+"/queue_size",
    (int)defaultOptions.queue_size);
  
  return this->getNodeHandle().subscribe(options);
}

ros::ServiceServer NodeImpl::advertiseService(const std::string& param,
    const ros::AdvertiseServiceOptions& defaultOptions) {
  std::string ns = std::string("servers/")+param;  
  ros::AdvertiseServiceOptions options = defaultOptions;
  options.service = getParam(ns+"/service", defaultOptions.service);
  
  return this->getNodeHandle().advertiseService(options);
}

ros::ServiceClient NodeImpl::serviceClient(const std::string& param, const
    ros::ServiceClientOptions& defaultOptions) {
  std::string ns = std::string("clients/")+param;  
  ros::ServiceClientOptions options = defaultOptions;
  options.service = getParam(ns+"/service", defaultOptions.service);
  options.persistent = getParam(ns+"/persistent", defaultOptions.persistent);
  
  return this->getNodeHandle().serviceClient(options);
}
    
}
