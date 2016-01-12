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

template <class S> bool ServiceClient::call(S& service) {
  typedef typename S::Request MReq;
  typedef typename S::Response MRes;
  
  return this->template call<MReq, MRes>(service.request, service.response);
}

template <class MReq, class MRes> bool ServiceClient::call(MReq& request,
    MRes& response) {
  if (impl) {
    bool result = impl->serviceClient.template call<MReq, MRes>(
      request, response);

    if (result)
      ++impl->numServedRequests;
    else
      ++impl->numFailedRequests;
    
    return result;
  }
  else
    return false;
}

}
