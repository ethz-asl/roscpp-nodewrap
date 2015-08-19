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

#include "roscpp_nodewrap/ServiceServer.h"
#include <roscpp_nodewrap/ServiceServerOptions.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ServiceServer::ServiceServer() {
}

ServiceServer::ServiceServer(const ServiceServer& src) :
  impl(src.impl) {
}

ServiceServer::~ServiceServer() {  
}

ServiceServer::Impl::Impl(const std::string& name, const NodeImplPtr& node) :
  name(name),
  numServedRequests(0),
  numFailedRequests(0),
  node(node) {
}

ServiceServer::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

ServiceServer::operator ros::ServiceServer() const {
  if (impl)
    return impl->serviceServer;
  else
    return ros::ServiceServer();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ServiceServer::getService() const {
  if (impl)
    return impl->serviceServer.getService();
  else
    return std::string();
}

const NodeImplPtr& ServiceServer::Impl::getNode() const {
  return node;
}

bool ServiceServer::Impl::isValid() const {
  return serviceServer && statusTask;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ServiceServer::shutdown() {
  if (impl)
    impl->shutdown();
}

void ServiceServer::Impl::init(const ServiceServerOptions& defaultOptions) {
  ros::AdvertiseServiceOptions options(defaultOptions);
  std::string ns = defaultOptions.ns.empty() ?
    ros::names::append("servers", name) : defaultOptions.ns;
  
  options.service = node->getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  serviceType = defaultOptions.datatype;
  serviceRequestType = defaultOptions.req_datatype;
  serviceResponseType = defaultOptions.res_datatype;
  serviceMd5Sum = defaultOptions.md5sum;
    
  ServiceServerCallbackHelperPtr helper =
    boost::dynamic_pointer_cast<ServiceServerCallbackHelper>(options.helper);
  if (helper)
    helper->serviceServer = shared_from_this();
  serviceServer = node->getNodeHandle().advertiseService(options);
  
  ServiceServerStatusTaskOptions statusTaskOptions(
    defaultOptions.statusTaskOptions);
  statusTaskOptions.ns = ros::names::append(ns,
    ros::names::append("diagnostics", "service_server_status"));
  statusTask = node->addDiagnosticTask<ServiceServerStatusTask>(
    std::string("ServiceServer ")+name+" Status", statusTaskOptions);
  statusTask.impl->as<ServiceServerStatusTask::Impl>().serviceServer =
    shared_from_this();
}

void ServiceServer::Impl::shutdown() {
  if (isValid()) {
    serviceServer = ros::ServiceServer();
    
    statusTask.shutdown();
  }
}

}
