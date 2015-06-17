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

#include "roscpp_nodewrap/Publisher.h"
#include <roscpp_nodewrap/PublisherOptions.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Publisher::Publisher() {
}

Publisher::Publisher(const Publisher& src) :
  impl(src.impl) {
}

Publisher::~Publisher() {  
}

Publisher::Impl::Impl(const std::string& name, const NodeImplPtr& node) :
  name(name),
  numPublishedMessages(0),
  node(node) {
}

Publisher::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Publisher::operator ros::Publisher() const {
  if (impl)
    return impl->publisher;
  else
    return ros::Publisher();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Publisher::getTopic() const {
  if (impl)
    return impl->publisher.getTopic();
  else
    return std::string();
}

size_t Publisher::getNumSubscribers() const {
  if (impl)
    return impl->publisher.getNumSubscribers();
  else
    return 0;
}

bool Publisher::isLatched() const {
  if (impl)
    return impl->publisher.isLatched();
  else
    return false;
}

const NodeImplPtr& Publisher::Impl::getNode() const {
  return node;
}

bool Publisher::Impl::isValid() const {
  return publisher && statusTask && publishingFrequencyTask &&
    messageStampFrequencyTask && (!messageHasHeader || messageLatencyTask);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Publisher::shutdown() {
  if (impl)
    impl->shutdown();
}

void Publisher::Impl::init(const PublisherOptions& defaultOptions) {
  ros::AdvertiseOptions options(defaultOptions);
  std::string ns = defaultOptions.ns.empty() ?
    ros::names::append("publishers", name) : defaultOptions.ns;
  
  options.topic = node->getParam(ros::names::append(ns, "topic"),
    defaultOptions.topic);
  options.queue_size = node->getParam(ros::names::append(ns, "queue_size"),
    (int)defaultOptions.queue_size);
  options.latch = node->getParam(ros::names::append(ns, "latch"),
    defaultOptions.latch);
  messageType = defaultOptions.datatype;
  messageHasHeader = defaultOptions.has_header;
  messageMd5Sum = defaultOptions.md5sum;
  messageQueueSize = defaultOptions.queue_size;
    
  publisher = node->getNodeHandle().advertise(options);
  
  PublisherStatusTaskOptions statusTaskOptions(
    defaultOptions.statusTaskOptions);
  statusTaskOptions.ns = ros::names::append(ns,
    ros::names::append("diagnostics", "publisher_status"));
  statusTask = node->addDiagnosticTask<PublisherStatusTask>(
    std::string("Publisher ")+name+" Status", statusTaskOptions);
  statusTask.impl->as<PublisherStatusTask::Impl>().publisher =
    shared_from_this();
  
  FrequencyTaskOptions publishingFrequencyTaskOptions(
    defaultOptions.publishingFrequencyTaskOptions);
  publishingFrequencyTaskOptions.ns = ros::names::append(
    ns, ros::names::append("diagnostics", "publishing_frequency"));
  publishingFrequencyTask = node->addDiagnosticTask<FrequencyTask>(
    std::string("Publisher ")+name+" Publishing Frequency",
    publishingFrequencyTaskOptions);
  
  FrequencyTaskOptions messageStampFrequencyTaskOptions(
    defaultOptions.messageStampFrequencyTaskOptions);
  messageStampFrequencyTaskOptions.ns = ros::names::append(
    ns, ros::names::append("diagnostics", "message_stamp_frequency"));
  messageStampFrequencyTask =
    node->addDiagnosticTask<MessageStampFrequencyTask>(
      std::string("Publisher ")+name+" Message Stamp Frequency",
      messageStampFrequencyTaskOptions);
  
  if (messageHasHeader) {
    LatencyTaskOptions messageLatencyTaskOptions(
      defaultOptions.messageLatencyTaskOptions);
    messageLatencyTaskOptions.ns = ros::names::append(
      ns, ros::names::append("diagnostics", "message_latency"));
    messageLatencyTask = node->addDiagnosticTask<MessageLatencyTask>(
      std::string("Publisher ")+name+" Message Latency",
      messageLatencyTaskOptions);
  }
}

void Publisher::Impl::shutdown() {
  if (isValid()) {
    publisher = ros::Publisher();
    
    statusTask.shutdown();
    publishingFrequencyTask.shutdown();
    messageStampFrequencyTask.shutdown();
    messageLatencyTask.shutdown();
  }
}

}
