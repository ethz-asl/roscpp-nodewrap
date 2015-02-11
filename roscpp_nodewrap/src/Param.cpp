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

#include "roscpp_nodewrap/Param.h"

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool Param<XmlRpc::XmlRpcValue>::fromXmlRpcValue(const XmlRpc::XmlRpcValue&
    xmlRpcValue, XmlRpc::XmlRpcValue& value) {
  value = xmlRpcValue;
  return true;
}

bool Param<XmlRpc::XmlRpcValue>::toXmlRpcValue(const XmlRpc::XmlRpcValue&
    value, XmlRpc::XmlRpcValue& xmlRpcValue) {
  xmlRpcValue = value;
  return true;
}

bool Param<XmlRpc::XmlRpcValue>::toRequest(const XmlRpc::XmlRpcValue& value,
    SetParamValueXml::Request& request) {
  request.value = value.toXml();
  return true;
}

bool Param<XmlRpc::XmlRpcValue>::fromResponse(const GetParamValueXml::Response&
    response, XmlRpc::XmlRpcValue& value) {
  int offset = 0;
  return value.fromXml(response.value, &offset);
}

bool Param<XmlRpc::XmlRpcValue>::fromRequest(const SetParamValueXml::Request&
    request, XmlRpc::XmlRpcValue& value) {
  int offset = 0;
  return value.fromXml(request.value, &offset);
}

bool Param<XmlRpc::XmlRpcValue>::toResponse(const XmlRpc::XmlRpcValue& value,
    GetParamValueXml::Response& response) {
  response.value = value.toXml();
  return true;
}

bool Param<std::string>::fromXmlRpcValue(const XmlRpc::XmlRpcValue&
    xmlRpcValue, std::string& value) {
  if (xmlRpcValue.getType() == XmlRpc::XmlRpcValue::TypeString) {
    value = const_cast<XmlRpc::XmlRpcValue&>(xmlRpcValue).operator
      std::string&();
    return true;
  }
  else
    return false;
}

bool Param<std::string>::toXmlRpcValue(const std::string& value,
    XmlRpc::XmlRpcValue& xmlRpcValue) {
  xmlRpcValue = XmlRpc::XmlRpcValue(value);
  return true;
}

bool Param<std::string>::fromRequest(const SetParamValueString::Request&
    request, std::string& value) {
  value = request.value;
  return true;
}

bool Param<std::string>::toRequest(const std::string& value,
    SetParamValueString::Request& request) {
  request.value = value;
  return true;
}

bool Param<std::string>::fromResponse(const GetParamValueString::Response&
    response, std::string& value) {
  value = response.value;
  return true;
}

bool Param<std::string>::toResponse(const std::string& value,
    GetParamValueString::Response& response) {
  response.value = value;
  return true;
}

bool Param<bool>::fromXmlRpcValue(const XmlRpc::XmlRpcValue& xmlRpcValue,
    bool& value) {
  if (xmlRpcValue.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    value = const_cast<XmlRpc::XmlRpcValue&>(xmlRpcValue).operator bool&();
    return true;
  }
  else
    return false;
}

bool Param<bool>::toXmlRpcValue(const bool& value, XmlRpc::XmlRpcValue&
    xmlRpcValue) {
  xmlRpcValue = XmlRpc::XmlRpcValue(value);
  return true;
}

bool Param<bool>::fromRequest(const SetParamValueBool::Request& request,
    bool& value) {
  value = request.value;
  return true;
}

bool Param<bool>::toRequest(const bool& value, SetParamValueBool::Request&
    request) {
  request.value = value;
  return true;
}

bool Param<bool>::fromResponse(const GetParamValueBool::Response& response,
    bool& value) {
  value = response.value;
  return true;
}

bool Param<bool>::toResponse(const bool& value, GetParamValueBool::Response&
    response) {
  response.value = value;
  return true;
}

}
