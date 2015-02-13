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

#include <roscpp_nodewrap/AdvertiseParamOptions.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class Spec> ParamServer::ImplT<Spec>::ImplT(const
    AdvertiseParamOptions& options, const NodeImplPtr& nodeImpl) :
  Impl(options, nodeImpl),
  callbacks(static_cast<const ParamServerCallbacksT<Spec>&>(
    *options.callbacks)) {
  ros::AdvertiseServiceOptions getParamValueOptions;
  getParamValueOptions.init<GetValueServiceRequest, GetValueServiceResponse>(
    ros::names::append(options.service, "get_value"),
    boost::bind(&ParamServer::ImplT<Spec>::getParamValueCallback,
    this, _1, _2));
  getParamValueOptions.callback_queue = options.callbackQueue;
  getParamValueOptions.tracked_object = options.trackedObject;
  this->getParamValueServer = this->advertise(getParamValueOptions);
  
  ros::AdvertiseServiceOptions setParamValueOptions;
  setParamValueOptions.init<SetValueServiceRequest, SetValueServiceResponse>(
    ros::names::append(options.service, "set_value"),
    boost::bind(&ParamServer::ImplT<Spec>::setParamValueCallback,
    this, _1, _2));
  setParamValueOptions.callback_queue = options.callbackQueue;
  setParamValueOptions.tracked_object = options.trackedObject;
  this->setParamValueServer = this->advertise(setParamValueOptions);
}

template <class Spec> ParamServer::ImplT<Spec>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class Spec> bool ParamServer::ImplT<Spec>::getParamValue(
    Value& value) {
  XmlRpc::XmlRpcValue xmlRpcValue;
  return this->getParamXmlRpcValue(xmlRpcValue) &&
    this->callbacks.fromXmlRpcValue(xmlRpcValue, value);
}

template <class Spec> bool ParamServer::ImplT<Spec>::setParamValue(
    const Value& value) {
  XmlRpc::XmlRpcValue xmlRpcValue;
  return this->callbacks.toXmlRpcValue(value, xmlRpcValue) &&
    this->setParamXmlRpcValue(xmlRpcValue);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class Spec> bool ParamServer::ImplT<Spec>::getParamValueCallback(
    GetValueServiceRequest& request, GetValueServiceResponse& response) {
  Value value;
  return this->getParamValue(value) &&
    this->callbacks.toResponse(value, response);
}

template <class Spec> bool ParamServer::ImplT<Spec>::setParamValueCallback(
    SetValueServiceRequest& request, SetValueServiceResponse& response) {
  Value value;
  return this->callbacks.fromRequest(request, value) &&
    this->setParamValue(value);
}

}
