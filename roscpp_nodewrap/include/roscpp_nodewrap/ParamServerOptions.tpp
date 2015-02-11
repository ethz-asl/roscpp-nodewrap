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

#include <roscpp_nodewrap/ParamServiceHelper.h>

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, class MGetReq, class MGetRes, class MSetReq,
  class MSetRes>
void ParamServerOptions::init(const std::string& service, const std::string&
      name, const boost::function<bool(const XmlRpc::XmlRpcValue&, T&)>&
      fromXmlRpcValue, const boost::function<bool(const T&,
      XmlRpc::XmlRpcValue&)>& toXmlRpcValue, const boost::function<bool(
      const MSetReq&, T&)>& fromRequest, const boost::function<bool(const T&,
      MGetRes&)>& toResponse, bool cached) {
  this->service = service;
  this->name = name;
  this->type = ParamType::template get<T>();
  this->cached = cached;
  this->helper.reset(new ParamServiceHelperT<
    ParamSpec<T, MGetReq, MGetRes, MSetReq, MSetRes> >(fromXmlRpcValue,
    toXmlRpcValue, fromRequest, toResponse));
}

template <typename T, class GetService, class SetService>
void ParamServerOptions::init(const std::string& service, const std::string&
    name, const boost::function<bool(const XmlRpc::XmlRpcValue&, T&)>&
    fromXmlRpcValue, const boost::function<bool(const T&,
    XmlRpc::XmlRpcValue&)>& toXmlRpcValue, const boost::function<bool(
    const typename SetService::Request&, T&)>& fromRequest, const
    boost::function<bool(const T&, typename GetService::Response&)>&
    toResponse, bool cached) {
  this->template init<T,
    typename GetService::Request,
    typename GetService::Response,
    typename SetService::Request,
    typename SetService::Response>(service, name, fromXmlRpcValue,
    toXmlRpcValue, fromRequest, toResponse, cached);
}

template <class Spec>
void ParamServerOptions::init(const std::string& service, const std::string&
    name, const typename Spec::FromXmlRpcValue& fromXmlRpcValue, const typename
    Spec::ToXmlRpcValue& toXmlRpcValue, const typename Spec::FromRequest&
    fromRequest, const typename Spec::ToResponse& toResponse, bool cached) {
  this->template init<typename Spec::Value,
    typename Spec::GetValueServiceRequest,
    typename Spec::GetValueServiceResponse,
    typename Spec::SetValueServiceRequest,
    typename Spec::SetValueServiceResponse>(service, name, fromXmlRpcValue,
    toXmlRpcValue, fromRequest, toResponse, cached);
}

template <typename T>
void ParamServerOptions::init(const std::string& service, const std::string&
    name, bool cached, typename boost::disable_if<typename boost::is_abstract<
    Param<T> > >::type* dummy) {
  this->template init<typename Param<T>::Spec>(service, name,
    boost::bind(&Param<T>::fromXmlRpcValue, _1, _2),
    boost::bind(&Param<T>::toXmlRpcValue, _1, _2),
    boost::bind(&Param<T>::fromRequest, _1, _2),
    boost::bind(&Param<T>::toResponse, _1, _2), cached);
}

}
