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

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsFloat<T> >::type>::fromXmlRpcValue(const
    XmlRpc::XmlRpcValue& xmlRpcValue, T& value) {
  if (xmlRpcValue.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    value = const_cast<XmlRpc::XmlRpcValue&>(xmlRpcValue).operator double&();
    return true;
  }
  else
    return false;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsFloat<T> >::type>::toXmlRpcValue(const T& value,
    XmlRpc::XmlRpcValue& xmlRpcValue) {
  xmlRpcValue = XmlRpc::XmlRpcValue((double)value);
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsFloat<T> >::type>::fromRequest(const
    SetParamValueDouble::Request& request, T& value) {
  value = request.value;
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsFloat<T> >::type>::toRequest(const T& value,
    SetParamValueDouble::Request& request) {
  request.value = value;
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsFloat<T> >::type>::fromResponse(const
    GetParamValueDouble::Response& response, T& value) {
  value = response.value;
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsFloat<T> >::type>::toResponse(const T& value,
    GetParamValueDouble::Response& response) {
  response.value = value;
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsInteger<T> >::type>::fromXmlRpcValue(const
    XmlRpc::XmlRpcValue& xmlRpcValue, T& value) {
  if (xmlRpcValue.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    value = const_cast<XmlRpc::XmlRpcValue&>(xmlRpcValue).operator int&();
    return true;
  }
  else
    return false;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsInteger<T> >::type>::toXmlRpcValue(const T& value,
    XmlRpc::XmlRpcValue& xmlRpcValue) {
  xmlRpcValue = XmlRpc::XmlRpcValue((int)value);
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsInteger<T> >::type>::fromRequest(const
    SetParamValueInt::Request& request, T& value) {
  value = request.value;
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsInteger<T> >::type>::toRequest(const T& value,
    SetParamValueInt::Request& request) {
  request.value = value;
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsInteger<T> >::type>::fromResponse(const
    GetParamValueInt::Response& response, T& value) {
  value = response.value;
  return true;
}

template <typename T> bool Param<T, typename boost::enable_if<typename
    ParamTraits::IsInteger<T> >::type>::toResponse(const T& value,
    GetParamValueInt::Response& response) {
  response.value = value;
  return true;
}

}
