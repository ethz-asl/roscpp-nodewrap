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

#include "roscpp_nodewrap/ServiceServer.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class MReq, class MRes>
ServiceServerCallbackHelperT<MReq, MRes>::ServiceServerCallbackHelperT(
    const Callback& callback) :
  helper(new ros::ServiceCallbackHelperT<Spec>(boost::bind(
    &ServiceServerCallbackHelperT<MReq, MRes>::wrappedCallback, this, _1, _2))),
  callback(callback) {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class MReq, class MRes>
bool ServiceServerCallbackHelperT<MReq, MRes>::call(
    ros::ServiceCallbackHelperCallParams& params) {
  return helper->call(params);
}

template <class MReq, class MRes>
bool ServiceServerCallbackHelperT<MReq, MRes>::wrappedCallback(
    MReq& request, MRes& response) {
  ServiceServer::ImplPtr serviceServerImpl =
    boost::static_pointer_cast<ServiceServer::Impl>(
      this->serviceServer.lock());
  
  if (serviceServerImpl) {
    ros::Time now = ros::Time::now();
    
    bool result = callback(request, response);
    
    if (result)
      ++serviceServerImpl->numServedRequests;
    else
      ++serviceServerImpl->numFailedRequests;
    
    return result;
  }
  else
    return callback(request, response);
}

}
