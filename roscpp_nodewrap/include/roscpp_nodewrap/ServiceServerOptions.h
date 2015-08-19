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

/** \file ServiceServerOptions.h
  * \brief Header file providing the ServiceServerOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_SERVICE_SERVER_OPTIONS_H
#define ROSCPP_NODEWRAP_SERVICE_SERVER_OPTIONS_H

#include <roscpp_nodewrap/diagnostics/ServiceServerStatusTaskOptions.h>

namespace nodewrap {
  /** \brief ROS service server wrapper options
    * 
    * This class encapsulates all options available for creating a ROS
    * service server wrapper.
    */
  class ServiceServerOptions :
    public ros::AdvertiseServiceOptions {
  public:
    /** \brief Default constructor
      */
    ServiceServerOptions();

    /** \brief Copy constructor
      */
    ServiceServerOptions(const ServiceServerOptions& src);
    
    /** \brief Copy constructor (ROS-compatible version)
      */
    ServiceServerOptions(const ros::AdvertiseServiceOptions& src);
    
    /** \brief Initialize the service server options, based on the
      *   service request/response type
      */
    template <class MReq, class MRes> void init(const std::string& service,
      const boost::function<bool(MReq&, MRes&)>& callback);
    
    /** \brief Initialize the service server options, based on the
      *   service type
      */
    template <class S> void init(const std::string& service, const
      boost::function<bool(typename S::Request&, typename S::Response&)>&
      callback);

    /** \brief The namespace of the service server options
      */
    std::string ns;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   status of the service server
      */ 
    ServiceServerStatusTaskOptions statusTaskOptions;
  };
};

#include <roscpp_nodewrap/ServiceServerOptions.tpp>

#endif
