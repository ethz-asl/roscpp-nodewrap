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

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/Subscriber.h"
#include "roscpp_nodewrap/SubscriberCallbackHelper.h"
#include <roscpp_nodewrap/SubscriberOptions.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Subscriber::Subscriber() {
}

Subscriber::Subscriber(const Subscriber& src) :
  impl(src.impl) {
}

Subscriber::~Subscriber() {  
}

Subscriber::Impl::Impl(const std::string& name, const NodeImplPtr& node) :
  name(name),
  numProcessedMessages(0),
  node(node) {
}

Subscriber::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Subscriber::operator ros::Subscriber() const {
  if (impl)
    return impl->subscriber;
  else
    return ros::Subscriber();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Subscriber::getTopic() const {
  if (impl)
    return impl->subscriber.getTopic();
  else
    return std::string();
}

size_t Subscriber::getNumPublishers() const {
  if (impl)
    return impl->subscriber.getNumPublishers();
  else
    return 0;
}

const NodeImplPtr& Subscriber::Impl::getNode() const {
  return node;
}

bool Subscriber::Impl::isValid() const {
  return subscriber && statusTask && processingFrequencyTask &&
    messageStampFrequencyTask && (!messageHasHeader || messageLatencyTask);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Subscriber::shutdown() {
  if (impl)
    impl->shutdown();
}

void Subscriber::Impl::init(const SubscriberOptions& defaultOptions) {
  ros::SubscribeOptions options(defaultOptions);
  std::string ns = defaultOptions.ns.empty() ?
    ros::names::append("subscribers", name) : defaultOptions.ns;
  
  options.topic = node->getParam(ros::names::append(ns, "topic"),
    defaultOptions.topic);
  options.queue_size = node->getParam(ros::names::append(ns, "queue_size"),
    (int)defaultOptions.queue_size);
  messageType = defaultOptions.datatype;
  messageHasHeader = defaultOptions.helper->hasHeader();
  messageMd5Sum = defaultOptions.md5sum;
  messageQueueSize = defaultOptions.queue_size;
  
  SubscriberCallbackHelperPtr helper =
    boost::dynamic_pointer_cast<SubscriberCallbackHelper>(options.helper);
  if (helper)
    helper->subscriber = shared_from_this();
  subscriber = node->getNodeHandle().subscribe(options);
  
  SubscriberStatusTaskOptions statusTaskOptions(
    defaultOptions.statusTaskOptions);
  statusTaskOptions.ns = ros::names::append(ns,
    ros::names::append("diagnostics", "subscriber_status"));
  statusTask = node->addDiagnosticTask<SubscriberStatusTask>(
    std::string("Subscriber ")+name+" Status", statusTaskOptions);
  statusTask.impl->as<SubscriberStatusTask::Impl>().subscriber =
    shared_from_this();
  
  FrequencyTaskOptions processingFrequencyTaskOptions(
    defaultOptions.processingFrequencyTaskOptions);
  processingFrequencyTaskOptions.ns = ros::names::append(
    ns, ros::names::append("diagnostics", "processing_frequency"));
  processingFrequencyTask = node->addDiagnosticTask<FrequencyTask>(
    std::string("Subscriber ")+name+" Processing Frequency",
    processingFrequencyTaskOptions);
  
  FrequencyTaskOptions messageStampFrequencyTaskOptions(
    defaultOptions.messageStampFrequencyTaskOptions);
  messageStampFrequencyTaskOptions.ns = ros::names::append(
    ns, ros::names::append("diagnostics", "message_stamp_frequency"));
  messageStampFrequencyTask =
    node->addDiagnosticTask<MessageStampFrequencyTask>(
      std::string("Subscriber ")+name+" Message Stamp Frequency",
      messageStampFrequencyTaskOptions);
  
  if (messageHasHeader) {
    LatencyTaskOptions messageLatencyTaskOptions(
      defaultOptions.messageLatencyTaskOptions);
    messageLatencyTaskOptions.ns = ros::names::append(
      ns, ros::names::append("diagnostics", "message_latency"));
    messageLatencyTask = node->addDiagnosticTask<MessageLatencyTask>(
      std::string("Subscriber ")+name+" Message Latency",
      messageLatencyTaskOptions);
  }
}

void Subscriber::Impl::shutdown() {
  if (isValid()) {
    subscriber = ros::Subscriber();
    
    statusTask.shutdown();
    processingFrequencyTask.shutdown();
    messageStampFrequencyTask.shutdown();
    messageLatencyTask.shutdown();
  }
}

}
