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

/** \file Subscriber.h
  * \brief Header file providing the Subscriber class interface
  */

#ifndef ROSCPP_NODEWRAP_SUBSCRIBER_H
#define ROSCPP_NODEWRAP_SUBSCRIBER_H

#include <roscpp_nodewrap/SubscriberImpl.h>

#include <roscpp_nodewrap/diagnostics/FrequencyTask.h>
#include <roscpp_nodewrap/diagnostics/MessageLatencyTask.h>
#include <roscpp_nodewrap/diagnostics/MessageStampFrequencyTask.h>
#include <roscpp_nodewrap/diagnostics/SubscriberStatusTask.h>

namespace nodewrap {
  /** \brief ROS subscriber wrapper
    * 
    * This class is a wrapper for native ROS subscriber. Its interface
    * is largely identical to that of the native ROS subscriber, but
    * provides additional accessors and diagnostics.
    */
  
  class Subscriber {
  friend class NodeImpl;
  template <class M> friend class SubscriberCallbackHelperT;
  friend class SubscriberStatusTask;
  public:
    /** \brief Default constructor
      */
    Subscriber();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source subscriber which is being copied
      *   to this subscriber.
      */
    Subscriber(const Subscriber& src);
    
    /** \brief Destructor
      */
    ~Subscriber();
    
    /** \brief Retrieve the topic this subscriber is subscribed to
      */
    std::string getTopic() const;
    
    /** \brief Retrieve this subscriber's number of publishers
      */
    size_t getNumPublishers() const;
    
    /** \brief Perform shutdown of the subscriber
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const Subscriber& subscriber) const {
      return (impl < subscriber.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const Subscriber& subscriber) const {
      return (impl == subscriber.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const Subscriber& subscriber) const {
      return (impl != subscriber.impl);
    };
    
    /** \brief Convert this wrapped subscriber to a native ROS subscriber
      */ 
    operator ros::Subscriber() const;
      
  private:
    /** \brief ROS subscriber wrapper implementation
      * 
      * This class provides the private implementation of the ROS
      * subscriber wrapper.
      */
    class Impl :
      public SubscriberImpl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const NodeImplPtr& node);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the node owning this subscriber
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief True, if this subscriber implementation is valid
        */ 
      bool isValid() const;
      
      /** \brief Initialize the subscriber
        */
      void init(const SubscriberOptions& defaultOptions);
            
      /** \brief Perform shutdown of the subscriber (implementation)
        */
      void shutdown();
            
      /** \brief The name of this subscriber
        */ 
      std::string name;
            
      /** \brief The type of messages subscribed to by this subscriber
        */ 
      std::string messageType;
      
      /** \brief If true, the message subscribed to by this subscriber has
        *   a message header
        */ 
      bool messageHasHeader;
      
      /** \brief The md5sum of the message type subscribed to by this
        *   subscriber
        */ 
      std::string messageMd5Sum;
      
      /** \brief The maximum number of incoming messages to be queued for
        *   processing
        */ 
      size_t messageQueueSize;
      
      /** \brief The number of messages processed by this subscriber
        */ 
      size_t numProcessedMessages;
      
      /** \brief The ROS subscriber
        */ 
      ros::Subscriber subscriber;
      
      /** \brief The node implementation owning this subscriber
        */ 
      NodeImplPtr node;
      
      /** \brief The diagnostic task for monitoring the status of this
        *   subscriber
        */ 
      SubscriberStatusTask statusTask;
      
      /** \brief The diagnostic task for monitoring the message processing
        *   frequency of this subscriber
        */ 
      FrequencyTask processingFrequencyTask;
      
      /** \brief The diagnostic task for monitoring the message timestamp
        *   frequency of this subscriber
        */ 
      MessageStampFrequencyTask messageStampFrequencyTask;
      
      /** \brief The diagnostic task for monitoring the message latency
        *   of this subscriber
        */ 
      MessageLatencyTask messageLatencyTask;
    };

    /** \brief Declaration of the subscriber implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the subscriber implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The subscriber's implementation
      */
    ImplPtr impl;
  };
};

#endif
