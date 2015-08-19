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

/** \file MessageLatencyTask.h
  * \brief Header file providing the MessageLatencyTask class
  *   interface
  */

#ifndef ROSCPP_NODEWRAP_MESSAGE_LATENCY_TASK_H
#define ROSCPP_NODEWRAP_MESSAGE_LATENCY_TASK_H

#include <roscpp_nodewrap/diagnostics/LatencyTask.h>

namespace nodewrap {
  /** \brief Diagnostic task for message latency monitoring
    * 
    * This class provides a diagnostic task to monitor the latency of
    * messages, i.e., the time difference between the message timestamp
    * and the occurrence of a cyclic event.
    */
  class MessageLatencyTask :
    public LatencyTask {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Forward declaration of the message latency task options
      */
    typedef LatencyTask::Options Options;
    
    /** \brief Default constructor
      */
    MessageLatencyTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source message latency task which is being
      *   copied to this message latency task.
      */
    MessageLatencyTask(const MessageLatencyTask& src);
    
    /** \brief Destructor
      */
    ~MessageLatencyTask();

    /** \brief Update the statistics of this message latency task
      * 
      * \param[in] message The message whose timestamp is used to update
      *   this message latency task.
      * \param[in] timeOfEvent The time of occurrence of the event.
      */
    template <typename M> void message(const boost::shared_ptr<M>& message,
      const ros::Time& timeOfEvent = ros::Time::now());
    
  private:
    /** \brief ROS message latency task implementation
      * 
      * This class provides the private implementation of the message
      * latency task.
      */
    class Impl :
      public LatencyTask::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
    };
    
    /** \brief ROS message latency task traits
      */
    template <typename M, class HasHeader = void> class HasHeaderTraits;
        
    /** \brief ROS message latency task traits (specialization for
      *   headerless messages)
      */
    template <typename M> class HasHeaderTraits<M, typename
        boost::disable_if<ros::message_traits::HasHeader<M> >::type> {
    public:
      static void message(const ImplPtr& impl, const boost::shared_ptr<M>&
        message, const ros::Time& timeOfEvent); 
    };
    
    /** \brief ROS message latency task traits (specialization for
      *   messages with header)
      */
    template <typename M> class HasHeaderTraits<M, typename
        boost::enable_if<ros::message_traits::HasHeader<M> >::type> {
    public:
      static void message(const ImplPtr& impl, const boost::shared_ptr<M>&
        message, const ros::Time& timeOfEvent); 
    };
  };
};

#include <roscpp_nodewrap/diagnostics/MessageLatencyTask.tpp>

#endif
