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

template <typename P> bool paramToValue(const ros::NodeHandle& nodeHandle,
    const std::string& key, P& value) {
  return nodeHandle.getParam(key, value);
}

template <typename P> void valueToParam(const ros::NodeHandle& nodeHandle,
    const P& value, const std::string& key) {
  nodeHandle.setParam(key, value);
}

template <typename P> void messageToValue(const typename
    ParamTraits<P>::SetParamValue::Request& message, P& value) {
  value = message.value;
}

template <typename P> void valueToMessage(const P& value,
    typename ParamTraits<P>::GetParamValue::Response& message) {
  message.value = value;
}

template <> void messageToValue<XmlRpc::XmlRpcValue>(const
    SetParamValueXml::Request& message, XmlRpc::XmlRpcValue& value) {
  int offset = 0;
  value.fromXml(message.value, &offset);
}

template <> void valueToMessage<XmlRpc::XmlRpcValue>(const
    XmlRpc::XmlRpcValue& value, GetParamValueXml::Response& message) {
  message.value = value.toXml();
}

}
