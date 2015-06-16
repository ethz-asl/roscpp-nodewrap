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

/** \file SubscriberCallbackHelper.h
  * \brief Header file providing the SubscriberCallbackHelper class interface
  */

#ifndef ROSCPP_NODEWRAP_SUBSCRIBER_CALLBACK_HELPER_H
#define ROSCPP_NODEWRAP_SUBSCRIBER_CALLBACK_HELPER_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS subscriber callback helper (abstract base)
    * 
    * This class is the abstract base of a helper class to deal with
    * subscription callbacks in the subscriber wrapper.
    */
  class SubscriberCallbackHelper :
    public ros::SubscriptionCallbackHelper {
  friend class Subscriber;  
  protected:    
    /** \brief The subscriber owning this callback helper
      */ 
    SubscriberImplWPtr subscriber;
  };
  
  /** \brief ROS subscriber callback helper (templated implementation)
    * 
    * This class is a helper class to deal with subscription callbacks
    * in the subscriber wrapper.
    */
  template <class M> class SubscriberCallbackHelperT :
    public SubscriberCallbackHelper {
  friend class Subscriber;
  public:
    /** \brief Forward declaration of the parameter type
      */
    typedef const boost::shared_ptr<M const>& Parameter;
    
    /** \brief Forward declaration of the callback type
      */
    typedef typename ros::SubscriptionCallbackHelperT<Parameter>::Callback
      Callback;
    
    /** \brief Constructor
      */
    SubscriberCallbackHelperT(const Callback& callback);

    const std::type_info& getTypeInfo();
    
    bool isConst();
    
    bool hasHeader();
    
    ros::VoidConstPtr deserialize(const
      ros::SubscriptionCallbackHelperDeserializeParams& params);
    
    void call(ros::SubscriptionCallbackHelperCallParams& params);

  private:
    /** \brief The native ROS subscription callback helper
      */ 
    ros::SubscriptionCallbackHelperPtr helper;
    
    /** \brief The callback to call when a message arrives
      */ 
    Callback callback;
    
    /** \brief The helper's callback wrapper
      */ 
    void wrappedCallback(Parameter message);
  };
};

#include <roscpp_nodewrap/SubscriberCallbackHelper.tpp>

#endif
