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

/** \file ServiceServerCallbackHelper.h
  * \brief Header file providing the ServiceServerCallbackHelper class
  *   interface
  */

#ifndef ROSCPP_NODEWRAP_SERVICE_SERVER_CALLBACK_HELPER_H
#define ROSCPP_NODEWRAP_SERVICE_SERVER_CALLBACK_HELPER_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS service server callback helper (abstract base)
    * 
    * This class is the abstract base of a helper class to deal with
    * service callbacks in the service server wrapper.
    */
  class ServiceServerCallbackHelper :
    public ros::ServiceCallbackHelper {
  friend class ServiceServer;  
  protected:    
    /** \brief The service server owning this callback helper
      */ 
    ServiceServerImplWPtr serviceServer;
  };
  
  /** \brief ROS service server callback helper (templated implementation)
    * 
    * This class is a helper class to deal with service callbacks
    * in the service server wrapper.
    */
  template <class MReq, class MRes> class ServiceServerCallbackHelperT :
    public ServiceServerCallbackHelper {
  friend class ServiceServer;
  public:
    /** \brief Forward declaration of the spec type
      */
    typedef ros::ServiceSpec<MReq, MRes> Spec;
    
    /** \brief Forward declaration of the callback type
      */
    typedef typename ros::ServiceCallbackHelperT<Spec>::Callback Callback;
    
    /** \brief Constructor
      */
    ServiceServerCallbackHelperT(const Callback& callback);

    bool call(ros::ServiceCallbackHelperCallParams& params);

  private:
    /** \brief The native ROS service callback helper
      */ 
    ros::ServiceCallbackHelperPtr helper;
    
    /** \brief The callback to call when the service is called
      */ 
    Callback callback;
    
    /** \brief The helper's callback wrapper
      */ 
    bool wrappedCallback(MReq& request, MRes& response);
  };
};

#include <roscpp_nodewrap/ServiceServerCallbackHelper.tpp>

#endif
