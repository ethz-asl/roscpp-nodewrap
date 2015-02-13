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

#include <roscpp_nodewrap/ParamServerHelper.h>

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, class MGetReq, class MGetRes, class MSetReq,
  class MSetRes>
void AdvertiseParamOptions::init(const std::string& service, const std::string&
      name, const ParamServerCallbacksT<ParamSpec<T, MGetReq, MGetRes, MSetReq,
      MSetRes> >& callbacks, bool cached) {
  this->service = service;
  this->name = name;
  this->type = ParamType::template get<T>();
  this->cached = cached;
  this->helper.reset(new ParamServerHelperT<
    ParamSpec<T, MGetReq, MGetRes, MSetReq, MSetRes> >());
  this->callbacks.reset(new ParamServerCallbacksT<
    ParamSpec<T, MGetReq, MGetRes, MSetReq, MSetRes> >(callbacks));
}

template <typename T, class GetService, class SetService>
void AdvertiseParamOptions::init(const std::string& service, const
    std::string& name, const ParamServerCallbacksT<typename ParamSpecS<T,
    GetService, SetService>::ToParamSpec>& callbacks, bool cached) {
  this->template init<T,
    typename GetService::Request,
    typename GetService::Response,
    typename SetService::Request,
    typename SetService::Response>(service, name, callbacks, cached);
}

template <class Spec>
void AdvertiseParamOptions::initBySpecType(const std::string& service, const
    std::string& name, const ParamServerCallbacksT<Spec>& callbacks, bool
    cached) {
  this->template init<typename Spec::Value,
    typename Spec::GetValueServiceRequest,
    typename Spec::GetValueServiceResponse,
    typename Spec::SetValueServiceRequest,
    typename Spec::SetValueServiceResponse>(service, name, callbacks, cached);
}

template <typename T>
void AdvertiseParamOptions::init(const std::string& service, const std::string&
    name, bool cached, typename boost::disable_if<typename boost::is_abstract<
    Param<T> > >::type* dummy) {
  ParamServerCallbacksT<typename Param<T>::Spec> callbacks(
    boost::bind(&Param<T>::fromXmlRpcValue, _1, _2),
    boost::bind(&Param<T>::toXmlRpcValue, _1, _2),
    boost::bind(&Param<T>::fromRequest, _1, _2),
    boost::bind(&Param<T>::toResponse, _1, _2));
  this->template initBySpecType<typename Param<T>::Spec>(service, name,
    callbacks, cached);
}

}
