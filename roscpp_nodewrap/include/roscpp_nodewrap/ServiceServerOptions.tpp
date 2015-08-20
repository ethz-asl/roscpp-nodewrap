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

#include "roscpp_nodewrap/ServiceServerCallbackHelper.h"

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class MReq, class MRes> void ServiceServerOptions::init(
    const std::string& service, const boost::function<bool(MReq&, MRes&)>&
    callback) {
  this->service = service;
  this->md5sum = ros::service_traits::template md5sum<MReq>();
  this->datatype = ros::service_traits::template datatype<MReq>();
  this->req_datatype = ros::message_traits::template datatype<MReq>();
  this->res_datatype = ros::message_traits::template datatype<MRes>();
  
  this->helper = ros::ServiceCallbackHelperPtr(new
    ServiceServerCallbackHelperT<MReq, MRes>(callback));
}

template <class S> void ServiceServerOptions::init(const std::string& service,
    const boost::function<bool(typename S::Request&, typename S::Response&)>&
    callback) {
  typedef typename S::Request MReq;
  typedef typename S::Response MRes;
  
  this->service = service;
  this->md5sum = ros::service_traits::template md5sum<S>();
  this->datatype = ros::service_traits::template datatype<S>();
  this->req_datatype = ros::message_traits::template datatype<MReq>();
  this->res_datatype = ros::message_traits::template datatype<MRes>();
  
  this->helper = ros::ServiceCallbackHelperPtr(new
    ServiceServerCallbackHelperT<MReq, MRes>(callback));
}

}
