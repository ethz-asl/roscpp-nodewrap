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

#include "roscpp_nodewrap/diagnostics/SubscriberStatusTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

SubscriberStatusTask::SubscriberStatusTask() {
}

SubscriberStatusTask::SubscriberStatusTask(const SubscriberStatusTask& src) :
  DiagnosticTask(src) {
}

SubscriberStatusTask::~SubscriberStatusTask() {
}
    
SubscriberStatusTask::Impl::Impl(const Options& defaultOptions, const
    std::string& name, const ManagerImplPtr& manager) :
  DiagnosticTask::Impl(name, manager),
  minNumPublishers(0) {
  std::string ns = defaultOptions.ns.empty() ? ros::names::append(
    ros::names::append("diagnostics", "subscriber_status"), name) :
    defaultOptions.ns;
  
  minNumPublishers = getNode()->getParam(
    ros::names::append(ns, "minimum_number_of_publishers"),
    (int)defaultOptions.minNumPublishers);
}
    
SubscriberStatusTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void SubscriberStatusTask::setSubscriber(const Subscriber& subscriber) {
  if (impl)
    impl->as<SubscriberStatusTask::Impl>().subscriber = subscriber.impl;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void SubscriberStatusTask::Impl::run(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  Subscriber::ImplPtr subscriberImpl =
    boost::static_pointer_cast<Subscriber::Impl>(subscriber.lock());
    
  if (subscriberImpl && subscriberImpl->isValid()) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "Subscriber is valid.");
    
    if (subscriberImpl->subscriber.getNumPublishers() >= minNumPublishers)
      status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Subscriber has %d publisher(s), at least %d required.",
        subscriberImpl->subscriber.getNumPublishers(), minNumPublishers);
    else
      status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Subscriber misses at least %d publisher(s).",
        minNumPublishers-subscriberImpl->subscriber.getNumPublishers());
      
    status.addf("Topic subscribed to", "%s",
      subscriberImpl->subscriber.getTopic().c_str());
    status.addf("Number of publishers", "%d",
      subscriberImpl->subscriber.getNumPublishers());
    status.addf("Message type", "%s", subscriberImpl->messageType.c_str());
    status.addf("Message has header", "%s",
      subscriberImpl->messageHasHeader ? "yes" : "no");
    status.addf("Message MD5 sum", "%s",
      subscriberImpl->messageMd5Sum.c_str());
    status.addf("Message queue size", "%d", subscriberImpl->messageQueueSize);
    status.addf("Number of messages processed", "%d",
      subscriberImpl->numProcessedMessages);
  }
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Subscriber is invalid.");
}

}
