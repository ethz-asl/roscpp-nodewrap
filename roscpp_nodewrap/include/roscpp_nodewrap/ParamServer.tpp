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
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename P> ParamServer::ParamServer(const std::string& key, const
    P& value, bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl) {
  this->template init<P>(key, value, cached, nodeImpl);
}

template <typename P, bool Cached> ParamServer::ParamServer(const std::string&
    key, const P& value, const boost::shared_ptr<NodeImpl>& nodeImpl) {
  this->template init<P, Cached>(key, value, nodeImpl);
}

template <typename P> ParamServer::ImplT<P>::ImplT(const std::string&
    key, bool cached, const SetValueCallback& setValueCallback, const
    GetValueCallback& getValueCallback, const boost::shared_ptr<NodeImpl>&
    nodeImpl) :
  Impl(key, static_cast<XmlRpc::XmlRpcValue::Type>(
    ParamTraits<P>::XmlRpcValueType), cached, nodeImpl) {
  ros::AdvertiseServiceOptions setParamValueOptions;
  setParamValueOptions.template init<typename SetValueService::Request,
    typename SetValueService::Response>(getNamespace()+"/set_value",
    setValueCallback);
  this->setParamValueServer = this->advertise(setParamValueOptions);
  
  ros::AdvertiseServiceOptions getParamValueOptions;
  getParamValueOptions.template init<typename GetValueService::Request,
    typename GetValueService::Response>(getNamespace()+"/get_value",
    getValueCallback);
  this->getParamValueServer = this->advertise(getParamValueOptions);
}

template <typename P> ParamServer::ImplT2<P, true>::ImplT2(const
    std::string& key, const P& value, const boost::shared_ptr<NodeImpl>&
    nodeImpl) :
  ImplT<P>(key, true,
    boost::bind(&ParamServer::ImplT2<P, true>::setParamValue, this, _1, _2),
    boost::bind(&ParamServer::ImplT2<P, true>::getParamValue, this, _1, _2),
    nodeImpl),
  value(value) {
}

template <typename P> ParamServer::ImplT2<P, false>::ImplT2(const
    std::string& key, const P& value, const boost::shared_ptr<NodeImpl>&
    nodeImpl) :
  ImplT<P>(key, false,
    boost::bind(&ParamServer::ImplT2<P, false>::setParamValue, this, _1, _2),
    boost::bind(&ParamServer::ImplT2<P, false>::getParamValue, this, _1, _2),
    nodeImpl) {
}

template <typename P> ParamServer::ImplT<P>::~ImplT() {
}

template <typename P> ParamServer::ImplT2<P, true>::~ImplT2() {
}

template <typename P> ParamServer::ImplT2<P, false>::~ImplT2() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename P> void ParamServer::init(const std::string& key, const
    P& value, bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl) {
  if (cached)
    this->template init<P, true>(key, value, nodeImpl);
  else
    this->template init<P, false>(key, value, nodeImpl);
}
    
template <typename P, bool Cached> void ParamServer::init(const std::string&
    key, const P& value, const boost::shared_ptr<NodeImpl>& nodeImpl) {
  this->impl.reset(new ParamServer::ImplT2<P, Cached>(key, value, nodeImpl)); 
}

template <typename P> bool ParamServer::ImplT2<P, true>::setParamValue(
    typename SetValueService::Request& request,
    typename SetValueService::Response& response) {
  messageToValue<P>(request, this->value);
  return true;
}

template <typename P> bool ParamServer::ImplT2<P, true>::getParamValue(
    typename GetValueService::Request& request,
    typename GetValueService::Response& response) {
  valueToMessage<P>(this->value, response);
  return true;
}

template <typename P> bool ParamServer::ImplT2<P, false>::setParamValue(
    typename SetValueService::Request& request,
    typename SetValueService::Response& response) {  
  P value;
  
  messageToValue<P>(request, value);
  valueToParam<P>(this->nodeImpl->getNodeHandle(), value, this->key);
  
  return true;
}

template <typename P> bool ParamServer::ImplT2<P, false>::getParamValue(
    typename GetValueService::Request& request,
    typename GetValueService::Response& response) {
  P value;
  
  if (paramToValue(this->nodeImpl->getNodeHandle(), this->key, value)) {
    valueToMessage<P>(value, response);
    return true;
  }
  else
    return false;
}

}
