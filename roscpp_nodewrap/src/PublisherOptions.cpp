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

#include "roscpp_nodewrap/PublisherOptions.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PublisherOptions::PublisherOptions() {
}

PublisherOptions::PublisherOptions(const PublisherOptions& src) :
  ros::AdvertiseOptions(src),
  ns(src.ns),
  statusTaskOptions(src.statusTaskOptions),
  publishingFrequencyTaskOptions(src.publishingFrequencyTaskOptions),
  messageStampFrequencyTaskOptions(src.messageStampFrequencyTaskOptions),
  messageLatencyTaskOptions(src.messageLatencyTaskOptions) {
}

PublisherOptions::PublisherOptions(const ros::AdvertiseOptions& src) :
  ros::AdvertiseOptions(src) {
}

}
