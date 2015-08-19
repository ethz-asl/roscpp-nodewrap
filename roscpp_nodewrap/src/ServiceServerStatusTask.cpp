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

#include "roscpp_nodewrap/diagnostics/ServiceServerStatusTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ServiceServerStatusTask::ServiceServerStatusTask() {
}

ServiceServerStatusTask::ServiceServerStatusTask(const
    ServiceServerStatusTask& src) :
  DiagnosticTask(src) {
}

ServiceServerStatusTask::~ServiceServerStatusTask() {
}
    
ServiceServerStatusTask::Impl::Impl(const Options& defaultOptions, const
    std::string& name, const ManagerImplPtr& manager) :
  DiagnosticTask::Impl(name, manager) {
}
    
ServiceServerStatusTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void ServiceServerStatusTask::setServiceServer(const ServiceServer&
    serviceServer) {
  if (impl)
    impl->as<ServiceServerStatusTask::Impl>().serviceServer =
      serviceServer.impl;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ServiceServerStatusTask::Impl::run(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  ServiceServer::ImplPtr serviceServerImpl =
    boost::static_pointer_cast<ServiceServer::Impl>(serviceServer.lock());
    
  if (serviceServerImpl && serviceServerImpl->isValid()) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "Service server is valid.");
    
    status.addf("Service advertised", "%s",
      serviceServerImpl->serviceServer.getService().c_str());
    status.addf("Service type", "%s", serviceServerImpl->serviceType.c_str());
    status.addf("Service request type", "%s",
      serviceServerImpl->serviceRequestType.c_str());
    status.addf("Service response type", "%s",
      serviceServerImpl->serviceResponseType.c_str());
    status.addf("Service MD5 sum", "%s",
      serviceServerImpl->serviceMd5Sum.c_str());
    status.addf("Number of requests served", "%d",
      serviceServerImpl->numServedRequests);
    status.addf("Number of requests failed", "%d",
      serviceServerImpl->numFailedRequests);
  }
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Service server is invalid.");
}

}
