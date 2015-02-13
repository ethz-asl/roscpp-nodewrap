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

#include <roscpp_nodewrap/Exceptions.h>

#include <roscpp_nodewrap/ParamClientOptions.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T> ParamClient::ImplI<T>::ImplI(const
    ParamClientOptions& options, const NodeImplPtr& nodeImpl) :
  Impl(options, nodeImpl) {
}

template <typename T> ParamClient::ImplI<T>::~ImplI() {
}

template <class Spec> ParamClient::ImplT<Spec>::ImplT(const
    ParamClientOptions& options, const NodeImplPtr& nodeImpl) :
  ImplI<Value> (options, nodeImpl),
  callbacks(static_cast<const ParamClientCallbacksT<Spec>&>(
    *options.callbacks)) {
  ros::ServiceClientOptions getParamValueOptions;
  getParamValueOptions.init<GetValueServiceRequest, GetValueServiceResponse>(
    ros::names::append(options.service, "get_value"), options.persistent,
    options.header);
  this->getParamValueClient = this->client(getParamValueOptions);
  
  ros::ServiceClientOptions setParamValueOptions;
  setParamValueOptions.init<SetValueServiceRequest, SetValueServiceResponse>(
    ros::names::append(options.service, "set_value"), options.persistent,
    options.header);
  this->setParamValueClient = this->client(setParamValueOptions);
}

template <class Spec> ParamClient::ImplT<Spec>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> bool ParamClient::setParamValue(const T& value,
    ros::Duration timeout) {
  if (impl && impl->type.template equals<T>() &&
      impl->setParamValueClient.waitForExistence(timeout))
    return static_cast<ImplI<T>&>(*impl).setParamValue(value);
  else
    return false;
}

template <typename T> bool ParamClient::getParamValue(T& value, ros::Duration
    timeout) {
  if (impl && impl->type.template equals<T>() &&
      impl->getParamValueClient.waitForExistence(timeout))
    return static_cast<ImplI<T>&>(*impl).getParamValue(value);
  else
    return false;
}

template <typename T> T ParamClient::getParamValue(ros::Duration timeout) {
  T value;
  
  if (impl) {
    if (impl->type.template equals<T>()) {
      if (impl->getParamValueClient.waitForExistence(timeout))
        static_cast<ImplI<T>&>(*impl).getParamValue(value);
    }
    else
      throw ParamTypeMismatchException(
        ParamType::template get<T>().getName(), impl->type.getName());
  }
    
  return value;
}

template <class Spec> bool ParamClient::ImplT<Spec>::getParamValue(Value&
    value) {
  GetValueServiceRequest getParamValueServiceRequest;
  GetValueServiceResponse getParamValueServiceResponse;
  
  return  this->getParamValueClient.call(getParamValueServiceRequest,
    getParamValueServiceResponse) && this->callbacks.fromResponse(
    getParamValueServiceResponse, value);
}

template <class Spec> bool ParamClient::ImplT<Spec>::setParamValue(
    const Value& value) {
  SetValueServiceRequest setParamValueServiceRequest;
  SetValueServiceResponse setParamValueServiceResponse;
  
  return  this->callbacks.toRequest(value, setParamValueServiceRequest) &&
    this->setParamValueClient.call(setParamValueServiceRequest,
    setParamValueServiceResponse);
}

}
