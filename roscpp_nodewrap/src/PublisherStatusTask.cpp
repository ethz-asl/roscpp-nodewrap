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

#include "roscpp_nodewrap/diagnostics/PublisherStatusTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PublisherStatusTask::PublisherStatusTask() {
}

PublisherStatusTask::PublisherStatusTask(const PublisherStatusTask& src) :
  DiagnosticTask(src) {
}

PublisherStatusTask::~PublisherStatusTask() {
}
    
PublisherStatusTask::Impl::Impl(const Options& defaultOptions, const
    std::string& name, const ManagerImplPtr& manager) :
  DiagnosticTask::Impl(name, manager),
  minNumSubscribers(0) {
  std::string ns = defaultOptions.ns.empty() ? ros::names::append(
    ros::names::append("diagnostics", "publisher_status"), name) :
    defaultOptions.ns;
  
  minNumSubscribers = getNode()->getParam(
    ros::names::append(ns, "minimum_number_of_subscribers"),
    (int)defaultOptions.minNumSubscribers);
}
    
PublisherStatusTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PublisherStatusTask::setPublisher(const Publisher& publisher) {
  if (impl)
    impl->as<PublisherStatusTask::Impl>().publisher = publisher.impl;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PublisherStatusTask::Impl::run(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  Publisher::ImplPtr publisherImpl =
    boost::static_pointer_cast<Publisher::Impl>(publisher.lock());
    
  if (publisherImpl && publisherImpl->isValid()) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "Publisher is valid.");
    
    if (publisherImpl->publisher.getNumSubscribers() >= minNumSubscribers)
      status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Publisher has %d subscriber(s), at least %d required.",
        publisherImpl->publisher.getNumSubscribers(), minNumSubscribers);
    else
      status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Publisher misses at least %d subscriber(s).",
        minNumSubscribers-publisherImpl->publisher.getNumSubscribers());
      
    status.addf("Topic published on", "%s",
      publisherImpl->publisher.getTopic().c_str());
    status.addf("Number of subscribers", "%d",
      publisherImpl->publisher.getNumSubscribers());
    status.addf("Topic is latched", "%s",
      publisherImpl->publisher.isLatched() ? "yes" : "no");
    status.addf("Message type", "%s", publisherImpl->messageType.c_str());
    status.addf("Message has header", "%s",
      publisherImpl->messageHasHeader ? "yes" : "no");
    status.addf("Message MD5 sum", "%s", publisherImpl->messageMd5Sum.c_str());
    status.addf("Message queue size", "%d", publisherImpl->messageQueueSize);
    status.addf("Number of messages published", "%d",
      publisherImpl->numPublishedMessages);
  }
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Publisher is invalid.");
}

}
