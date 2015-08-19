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

/** \file Forwards.h
  * \brief Header file providing forward declarations for the node wrapper
  *   (and its dependencies)
  */

#ifndef ROSCPP_NODEWRAP_FORWARDS_H
#define ROSCPP_NODEWRAP_FORWARDS_H

#include <ros/forwards.h>

#include <roscpp_nodewrap/diagnostics/DiagnosticsForwards.h>
#include <roscpp_nodewrap/timer/TimerForwards.h>
#include <roscpp_nodewrap/worker/WorkerForwards.h>

namespace roscpp_nodewrap_msgs {};
  
namespace nodewrap {
  using namespace roscpp_nodewrap_msgs;
  
  /** \brief Forward declaration of the node implementation
    */
  class NodeImpl;  
  /** \brief Forward declaration of the node implementation pointer type
    */
  typedef boost::shared_ptr<NodeImpl> NodeImplPtr;
  /** \brief Forward declaration of the node implementation weak pointer type
    */
  typedef boost::weak_ptr<NodeImpl> NodeImplWPtr;
  
  /** \brief Forward declaration of the publisher options
    */
  class PublisherOptions;
  /** \brief Forward declaration of the subscriber options
    */
  class SubscriberOptions;
  /** \brief Forward declaration of the service server options
    */
  class ServiceServerOptions;
  /** \brief Forward declaration of the service client options
    */
  class ServiceClientOptions;
  
  /** \brief Forward declaration of the subscriber callback helper
    */
  class SubscriberCallbackHelper;
  /** \brief Forward declaration of the subscriber callback helper
    *   pointer type
    */
  typedef boost::shared_ptr<SubscriberCallbackHelper>
    SubscriberCallbackHelperPtr;
  
  /** \brief Forward declaration of the service server callback helper
    */
  class ServiceServerCallbackHelper;
  /** \brief Forward declaration of the service server callback helper
    *   pointer type
    */
  typedef boost::shared_ptr<ServiceServerCallbackHelper>
    ServiceServerCallbackHelperPtr;
  
  /** \brief Forward declaration of the publisher
    */
  class Publisher;  
  /** \brief Forward declaration of the publisher pointer type
    */
  typedef boost::shared_ptr<Publisher> PublisherPtr;
  /** \brief Forward declaration of the publisher weak pointer type
    */
  typedef boost::weak_ptr<Publisher> PublisherWPtr;
  
  /** \brief Forward declaration of the publisher implementation
    */
  class PublisherImpl;  
  /** \brief Forward declaration of the publisher implementation
    *   pointer type
    */
  typedef boost::shared_ptr<PublisherImpl> PublisherImplPtr;
  /** \brief Forward declaration of the publisher implementation
    *   weak pointer type
    */
  typedef boost::weak_ptr<PublisherImpl> PublisherImplWPtr;
  
  /** \brief Forward declaration of the subscriber
    */
  class Subscriber;  
  /** \brief Forward declaration of the subscriber pointer type
    */
  typedef boost::shared_ptr<Subscriber> SubscriberPtr;
  /** \brief Forward declaration of the subscriber weak pointer type
    */
  typedef boost::weak_ptr<Subscriber> SubscriberWPtr;
  
  /** \brief Forward declaration of the subscriber implementation
    */
  class SubscriberImpl;  
  /** \brief Forward declaration of the subscriber implementation
    *   pointer type
    */
  typedef boost::shared_ptr<SubscriberImpl> SubscriberImplPtr;
  /** \brief Forward declaration of the subscriber implementation
    *   weak pointer type
    */
  typedef boost::weak_ptr<SubscriberImpl> SubscriberImplWPtr;
  
  /** \brief Forward declaration of the service server
    */
  class ServiceServer;
  /** \brief Forward declaration of the service server pointer type
    */
  typedef boost::shared_ptr<ServiceServer> ServiceServerPtr;
  /** \brief Forward declaration of the service server weak pointer type
    */
  typedef boost::weak_ptr<ServiceServer> ServiceServerWPtr;
  
  /** \brief Forward declaration of the service server implementation
    */
  class ServiceServerImpl;  
  /** \brief Forward declaration of the service server implementation
    *   pointer type
    */
  typedef boost::shared_ptr<ServiceServerImpl> ServiceServerImplPtr;
  /** \brief Forward declaration of the service server implementation
    *   weak pointer type
    */
  typedef boost::weak_ptr<ServiceServerImpl> ServiceServerImplWPtr;
  
  /** \brief Forward declaration of the service client
    */
  class ServiceClient;
  /** \brief Forward declaration of the service client pointer type
    */
  typedef boost::shared_ptr<ServiceClient> ServiceClientPtr;
  /** \brief Forward declaration of the service client weak pointer type
    */
  typedef boost::weak_ptr<ServiceClient> ServiceClientWPtr;
  
  /** \brief Forward declaration of the service client implementation
    */
  class ServiceClientImpl;  
  /** \brief Forward declaration of the service client implementation
    *   pointer type
    */
  typedef boost::shared_ptr<ServiceClientImpl> ServiceClientImplPtr;
  /** \brief Forward declaration of the service client implementation
    *   weak pointer type
    */
  typedef boost::weak_ptr<ServiceClientImpl> ServiceClientImplWPtr;
};

#endif
