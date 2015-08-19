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

/** \file ServiceClientOptions.h
  * \brief Header file providing the ServiceClientOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_SERVICE_CLIENT_OPTIONS_H
#define ROSCPP_NODEWRAP_SERVICE_CLIENT_OPTIONS_H

#include <roscpp_nodewrap/diagnostics/ServiceClientStatusTaskOptions.h>

namespace nodewrap {
  /** \brief ROS publisher wrapper options
    * 
    * This class encapsulates all options available for creating a ROS
    * publisher wrapper.
    */
  class ServiceClientOptions :
    public ros::ServiceClientOptions {
  public:
    /** \brief Default constructor
      */
    ServiceClientOptions();

    /** \brief Copy constructor
      */
    ServiceClientOptions(const ServiceClientOptions& src);
    
    /** \brief Copy constructor (ROS-compatible version)
      */
    ServiceClientOptions(const ros::ServiceClientOptions& src);
    
    /** \brief Initialize the service client options, based on the
      *   service request/response type
      */
    template <class MReq, class MRes> void init(const std::string& service,
      bool persistent, const ros::M_string& header);
    
    /** \brief Initialize the service client options, based on the
      *   service type
      */
    template <class S> void init(const std::string& service, bool persistent,
      const ros::M_string& header);
    
    /** \brief The namespace of the service client options
      */
    std::string ns;
    
    /** \brief The type of service connected to by the service client
      */ 
    std::string datatype;
    
    /** \brief The request message type of the service connected to by
      *   the service client
      */ 
    std::string req_datatype;
    
    /** \brief The response message type of the service connected to by
      *   the service client
      */ 
    std::string res_datatype;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   status of the service client
      */ 
    ServiceClientStatusTaskOptions statusTaskOptions;
  };
};

#include <roscpp_nodewrap/ServiceClientOptions.tpp>

#endif
