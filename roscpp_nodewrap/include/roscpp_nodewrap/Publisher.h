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

/** \file Publisher.h
  * \brief Header file providing the Publisher class interface
  */

#ifndef ROSCPP_NODEWRAP_PUBLISHER_H
#define ROSCPP_NODEWRAP_PUBLISHER_H

#include <roscpp_nodewrap/PublisherImpl.h>

#include <roscpp_nodewrap/diagnostics/FrequencyTask.h>
#include <roscpp_nodewrap/diagnostics/MessageLatencyTask.h>
#include <roscpp_nodewrap/diagnostics/MessageStampFrequencyTask.h>
#include <roscpp_nodewrap/diagnostics/PublisherStatusTask.h>

namespace nodewrap {
  /** \brief ROS publisher wrapper
    * 
    * This class is a wrapper for native ROS publisher. Its interface
    * is largely identical to that of the native ROS publisher, but
    * provides additional accessors and diagnostics.
    */
  
  class Publisher {
  friend class NodeImpl;
  friend class PublisherStatusTask;
  public:
    /** \brief Default constructor
      */
    Publisher();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source publisher which is being copied
      *   to this publisher.
      */
    Publisher(const Publisher& src);
    
    /** \brief Destructor
      */
    ~Publisher();
    
    /** \brief Retrieve the topic this publisher publishes on
      */
    std::string getTopic() const;
    
    /** \brief Retrieve this publisher's number of subscribers
      */
    size_t getNumSubscribers() const;
    
    /** \brief True, if the topic this publisher publishes on is latched
      */
    bool isLatched() const;
    
    /** \brief Perform shutdown of the publisher
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const Publisher& publisher) const {
      return (impl < publisher.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const Publisher& publisher) const {
      return (impl == publisher.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const Publisher& publisher) const {
      return (impl != publisher.impl);
    };
    
    /** \brief Publish a message on the topic associated with this publisher
      *
      * \note The wrapped publisher enforces use of the const message pointer
      *   signature.
      */ 
    template <typename M> void publish(const boost::shared_ptr<M>& message)
      const;
  
    /** \brief Convert this wrapped publisher to a native ROS publisher
      */ 
    operator ros::Publisher() const;
      
  private:
    /** \brief ROS publisher wrapper implementation
      * 
      * This class provides the private implementation of the ROS
      * publisher wrapper.
      */
    class Impl :
      public PublisherImpl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const NodeImplPtr& node);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the node owning this publisher
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief True, if this publisher implementation is valid
        */ 
      bool isValid() const;
      
      /** \brief Initialize the publisher
        */
      void init(const PublisherOptions& defaultOptions);
            
      /** \brief Perform shutdown of the publisher (implementation)
        */
      void shutdown();
            
      /** \brief The name of this publisher
        */ 
      std::string name;
            
      /** \brief The type of messages published by this subscriber
        */ 
      std::string messageType;
      
      /** \brief If true, the message published by this subscriber has
        *   a message header
        */ 
      bool messageHasHeader;
      
      /** \brief The md5sum of the message type published by this
        *   subscriber
        */ 
      std::string messageMd5Sum;
      
      /** \brief The maximum number of outgoing messages to be queued for
        *   delivery
        */ 
      size_t messageQueueSize;
      
      /** \brief The number of messages published by this publisher
        */ 
      size_t numPublishedMessages;
      
      /** \brief The ROS publisher
        */ 
      ros::Publisher publisher;
      
      /** \brief The node implementation owning this publisher
        */ 
      NodeImplPtr node;
      
      /** \brief The diagnostic task for monitoring the status of this
        *   publisher
        */ 
      PublisherStatusTask statusTask;
      
      /** \brief The diagnostic task for monitoring the message publishing
        *   frequency of this publisher
        */ 
      FrequencyTask publishingFrequencyTask;
      
      /** \brief The diagnostic task for monitoring the message timestamp
        *   frequency of this publisher
        */ 
      MessageStampFrequencyTask messageStampFrequencyTask;
      
      /** \brief The diagnostic task for monitoring the message latency
        *   of this publisher
        */ 
      MessageLatencyTask messageLatencyTask;
    };

    /** \brief Declaration of the publisher implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the publisher implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The publisher's implementation
      */
    ImplPtr impl;
  };
};

#include <roscpp_nodewrap/Publisher.tpp>

#endif
