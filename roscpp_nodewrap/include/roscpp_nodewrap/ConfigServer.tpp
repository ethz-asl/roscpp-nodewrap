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

template <typename P> ParamServer ConfigServer::advertiseParam(const
    std::string& key, const std::string& name, const std::string& service,
    bool cached) {
  if (impl)
    return impl->template advertiseParam<P>(key, name, service, cached);
  else
    return ParamServer();
}

template <typename P> ParamServer ConfigServer::Impl::advertiseParam(const
    std::string& key, const std::string& name, const std::string& service,
    bool cached) {
  AdvertiseParamOptions options;
  options.template init<P>(service.empty() ? ros::names::append(this->service,
    ros::names::append("params", key)) : service, name.empty() ? key : name,
    cached);
  
  return this->advertiseParam(key, options);
}

}
