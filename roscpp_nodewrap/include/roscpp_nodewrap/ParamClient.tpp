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

#include <roscpp_nodewrap/ParamClientOptions.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class Spec> ParamClient::ImplT<Spec>::ImplT(const FromResponse&
    fromResponse, const ToRequest& toRequest, const ParamClientOptions&
    options, const NodeImplPtr& nodeImpl) :
  Impl(options, nodeImpl),
  fromResponse(fromResponse),
  toRequest(toRequest) {
  std::string ns = ros::names::append("params", options.service);
  
  ros::ServiceClientOptions getParamValueOptions;
  getParamValueOptions.init<GetValueServiceRequest, GetValueServiceResponse>(
    ros::names::append(ns, "get_value"), options.persistent, options.header);
  getParamValueOptions.header = options.header;
  getParamValueOptions.persistent = options.persistent;
  this->getParamValueClient = this->client(getParamValueOptions);
  
  ros::ServiceClientOptions setParamValueOptions;
  setParamValueOptions.init<SetValueServiceRequest, SetValueServiceResponse>(
    ros::names::append(ns, "set_value"), options.persistent, options.header);
  setParamValueOptions.header = options.header;
  setParamValueOptions.persistent = options.persistent;
  this->setParamValueClient = this->client(setParamValueOptions);
}

template <class Spec> ParamClient::ImplT<Spec>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> bool ParamClient::setParamValue(const T& value) {
  return false;
}

template <typename T> bool ParamClient::getParamValue(T& value) {
  return false;
}

template <class Spec> bool ParamClient::ImplT<Spec>::getParamValue(
    Value& value) {
  return false;
}

template <class Spec> bool ParamClient::ImplT<Spec>::setParamValue(
    const Value& value) {
  return false;
}

}
