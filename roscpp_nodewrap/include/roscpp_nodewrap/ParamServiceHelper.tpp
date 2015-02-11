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

#include <roscpp_nodewrap/ParamClient.h>
#include <roscpp_nodewrap/ParamServer.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/
    
template <class Spec> ParamServiceHelperT<Spec>::ParamServiceHelperT(const
    FromResponse& fromResponse, const ToRequest& toRequest) :
  toRequest(toRequest),
  fromResponse(fromResponse) {
}

template <class Spec> ParamServiceHelperT<Spec>::ParamServiceHelperT(const
    FromXmlRpcValue& fromXmlRpcValue, const ToXmlRpcValue& toXmlRpcValue,
    const FromRequest& fromRequest, const ToResponse& toResponse) :
  fromXmlRpcValue(fromXmlRpcValue),
  toXmlRpcValue(toXmlRpcValue),
  fromRequest(fromRequest),
  toResponse(toResponse) {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class Spec>
ParamClient ParamServiceHelperT<Spec>::createClient(const ParamClientOptions&
    options, const NodeImplPtr& nodeImpl) {
  ParamClient paramClient;
  paramClient.impl.reset(new ParamClient::ImplT<Spec>(fromResponse,
    toRequest, options, nodeImpl));
  
  return paramClient;
}

template <class Spec>
ParamServer ParamServiceHelperT<Spec>::createServer(const ParamServerOptions&
    options, const NodeImplPtr& nodeImpl) {
  ParamServer paramServer;
  paramServer.impl.reset(new ParamServer::ImplT<Spec>(fromXmlRpcValue,
    toXmlRpcValue, fromRequest, toResponse, options, nodeImpl));
  
  return paramServer;
}

}
