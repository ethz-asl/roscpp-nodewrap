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

/** \file MessageStampFrequencyTask.h
  * \brief Header file providing the MessageStampFrequencyTask class
  *   interface
  */

#ifndef ROSCPP_NODEWRAP_MESSAGE_STAMP_FREQUENCY_TASK_H
#define ROSCPP_NODEWRAP_MESSAGE_STAMP_FREQUENCY_TASK_H

#include <roscpp_nodewrap/diagnostics/FrequencyTask.h>

namespace nodewrap {
  /** \brief Diagnostic task for message timestamp frequency monitoring
    * 
    * This class provides a diagnostic task to monitor the frequency of
    * message timestamps.
    */
  class MessageStampFrequencyTask :
    public FrequencyTask {
  friend class DiagnosticTaskManager;
  public:
    /** \brief Forward declaration of the message timestamp frequency
      *   task options
      */
    typedef FrequencyTask::Options Options;
    
    /** \brief Default constructor
      */
    MessageStampFrequencyTask();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source message timestamp frequency task which is
      *   being copied to this message timestamp frequency task.
      */
    MessageStampFrequencyTask(const MessageStampFrequencyTask& src);
    
    /** \brief Destructor
      */
    ~MessageStampFrequencyTask();

    /** \brief Update the statistics of this message timestamp frequency task
      * 
      * \param[in] message The message whose timestamp is used to update
      *   this message timestamp frequency task.
      */
    template <typename M> void message(const boost::shared_ptr<M>& message);
    
  private:
    /** \brief ROS message timestamp frequency task implementation
      * 
      * This class provides the private implementation of the message
      * timestamp frequency task.
      */
    class Impl :
      public FrequencyTask::Impl {
    public:        
      /** \brief Constructor
        */
      Impl(const Options& defaultOptions, const std::string& name, const
        ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      ~Impl();
    };
    
    /** \brief ROS message timestamp frequency task traits
      */
    template <typename M, class HasHeader = void> class HasHeaderTraits;
        
    /** \brief ROS message timestamp frequency task traits (specialization
      *   for headerless messages)
      */
    template <typename M> class HasHeaderTraits<M, typename
        boost::disable_if<ros::message_traits::HasHeader<M> >::type> {
    public:
      static void message(const ImplPtr& impl, const boost::shared_ptr<M>&
        message); 
    };
    
    /** \brief ROS message timestamp frequency task traits (specialization
      *   for messages with header)
      */
    template <typename M> class HasHeaderTraits<M, typename
        boost::enable_if<ros::message_traits::HasHeader<M> >::type> {
    public:
      static void message(const ImplPtr& impl, const boost::shared_ptr<M>&
        message); 
    };
  };
};

#include <roscpp_nodewrap/diagnostics/MessageStampFrequencyTask.tpp>

#endif
