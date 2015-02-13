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

#include <roscpp_nodewrap/ParamClientHelper.h>

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, class MGetReq, class MGetRes, class MSetReq,
    class MSetRes>
void ParamClientOptions::init(const std::string& service, const
    ParamClientCallbacksT<ParamSpec<T, MGetReq, MGetRes, MSetReq, MSetRes> >&
    callbacks, bool persistent) {
  this->service = service;
  this->type = ParamType::template get<T>();
  this->persistent = persistent;
  this->helper.reset(new ParamClientHelperT<
    ParamSpec<T, MGetReq, MGetRes, MSetReq, MSetRes> >());
  this->callbacks.reset(new ParamClientCallbacksT<
    ParamSpec<T, MGetReq, MGetRes, MSetReq, MSetRes> >(callbacks));
}

template <typename T, class GetService, class SetService>
void ParamClientOptions::init(const std::string& service, const
    ParamClientCallbacksT<typename ParamSpecS<T, GetService,
    SetService>::ToParamSpec>& callbacks, bool persistent) {
  this->template init<T,
    typename GetService::Request,
    typename GetService::Response,
    typename SetService::Request,
    typename SetService::Response>(service, callbacks, persistent);
}

template <class Spec>
void ParamClientOptions::initBySpecType(const std::string& service, const
    ParamClientCallbacksT<Spec>& callbacks, bool persistent) {
  this->template init<typename Spec::Value,
    typename Spec::GetValueServiceRequest,
    typename Spec::GetValueServiceResponse,
    typename Spec::SetValueServiceRequest,
    typename Spec::SetValueServiceResponse>(service, callbacks, persistent);
}

template <typename T>
void ParamClientOptions::init(const std::string& service, bool persistent,
    typename boost::disable_if<typename boost::is_abstract<Param<T> > >::type*
    dummy) {
  ParamClientCallbacksT<typename Param<T>::Spec> callbacks(
    boost::bind(&Param<T>::fromResponse, _1, _2),
    boost::bind(&Param<T>::toRequest, _1, _2));
  this->template initBySpecType<typename Param<T>::Spec>(service, callbacks,
    persistent);
}

}
