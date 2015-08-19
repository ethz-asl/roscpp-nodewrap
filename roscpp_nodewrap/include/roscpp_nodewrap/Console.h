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

/** \file Console.h
  * \brief Header file defining ROS console wrapper macros
  * 
  * To allow for portable implementations of nodes and nodelets, use of the
  * ROS console wrapper macros provided by this header is crucial.
  */

#ifndef ROSCPP_NODEWRAP_CONSOLE_H
#define ROSCPP_NODEWRAP_CONSOLE_H

#include <ros/console.h>

#define NODEWRAP_DEBUG(...) if (this->isNodelet()) ROS_DEBUG_NAMED(this->getName(), __VA_ARGS__); else ROS_DEBUG(__VA_ARGS__)
#define NODEWRAP_DEBUG_STREAM(...) if (this->isNodelet()) ROS_DEBUG_STREAM_NAMED(this->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM(__VA_ARGS__)
#define NODEWRAP_DEBUG_ONCE(...) if (this->isNodelet()) ROS_DEBUG_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_DEBUG_ONCE(__VA_ARGS__)
#define NODEWRAP_DEBUG_STREAM_ONCE(...) if (this->isNodelet()) ROS_DEBUG_STREAM_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_DEBUG_COND(cond, ...) if (this->isNodelet()) ROS_DEBUG_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_DEBUG_COND(cond, __VA_ARGS__)
#define NODEWRAP_DEBUG_STREAM_COND(cond, ...) if (this->isNodelet()) ROS_DEBUG_STREAM_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_DEBUG_THROTTLE(rate, ...) if (this->isNodelet()) ROS_DEBUG_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_DEBUG_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_DEBUG_STREAM_THROTTLE(rate, ...) if (this->isNodelet()) ROS_DEBUG_STREAM_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_DEBUG_FILTER(filter, ...) if (this->isNodelet()) ROS_DEBUG_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_DEBUG_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_DEBUG_STREAM_FILTER(filter, ...) if (this->isNodelet()) ROS_DEBUG_STREAM_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_INFO(...) if (this->isNodelet()) ROS_INFO_NAMED(this->getName(), __VA_ARGS__); else ROS_INFO(__VA_ARGS__)
#define NODEWRAP_INFO_STREAM(...) if (this->isNodelet()) ROS_INFO_STREAM_NAMED(this->getName(), __VA_ARGS__); else ROS_INFO_STREAM(__VA_ARGS__)
#define NODEWRAP_INFO_ONCE(...) if (this->isNodelet()) ROS_INFO_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_INFO_ONCE(__VA_ARGS__)
#define NODEWRAP_INFO_STREAM_ONCE(...) if (this->isNodelet()) ROS_INFO_STREAM_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_INFO_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_INFO_COND(cond, ...) if (this->isNodelet()) ROS_INFO_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_INFO_COND(cond, __VA_ARGS__)
#define NODEWRAP_INFO_STREAM_COND(cond, ...) if (this->isNodelet()) ROS_INFO_STREAM_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_INFO_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_INFO_THROTTLE(rate, ...) if (this->isNodelet()) ROS_INFO_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_INFO_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_INFO_STREAM_THROTTLE(rate, ...) if (this->isNodelet()) ROS_INFO_STREAM_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_INFO_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_INFO_FILTER(filter, ...) if (this->isNodelet()) ROS_INFO_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_INFO_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_INFO_STREAM_FILTER(filter, ...) if (this->isNodelet()) ROS_INFO_STREAM_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_INFO_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_WARN(...) if (this->isNodelet()) ROS_WARN_NAMED(this->getName(), __VA_ARGS__); else ROS_WARN(__VA_ARGS__)
#define NODEWRAP_WARN_STREAM(...) if (this->isNodelet()) ROS_WARN_STREAM_NAMED(this->getName(), __VA_ARGS__); else ROS_WARN_STREAM(__VA_ARGS__)
#define NODEWRAP_WARN_ONCE(...) if (this->isNodelet()) ROS_WARN_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_WARN_ONCE(__VA_ARGS__)
#define NODEWRAP_WARN_STREAM_ONCE(...) if (this->isNodelet()) ROS_WARN_STREAM_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_WARN_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_WARN_COND(cond, ...) if (this->isNodelet()) ROS_WARN_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_WARN_COND(cond, __VA_ARGS__)
#define NODEWRAP_WARN_STREAM_COND(cond, ...) if (this->isNodelet()) ROS_WARN_STREAM_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_WARN_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_WARN_THROTTLE(rate, ...) if (this->isNodelet()) ROS_WARN_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_WARN_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_WARN_STREAM_THROTTLE(rate, ...) if (this->isNodelet()) ROS_WARN_STREAM_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_WARN_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_WARN_FILTER(filter, ...) if (this->isNodelet()) ROS_WARN_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_WARN_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_WARN_STREAM_FILTER(filter, ...) if (this->isNodelet()) ROS_WARN_STREAM_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_WARN_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_ERROR(...) if (this->isNodelet()) ROS_ERROR_NAMED(this->getName(), __VA_ARGS__); else ROS_ERROR(__VA_ARGS__)
#define NODEWRAP_ERROR_STREAM(...) if (this->isNodelet()) ROS_ERROR_STREAM_NAMED(this->getName(), __VA_ARGS__); else ROS_ERROR_STREAM(__VA_ARGS__)
#define NODEWRAP_ERROR_ONCE(...) if (this->isNodelet()) ROS_ERROR_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_ERROR_ONCE(__VA_ARGS__)
#define NODEWRAP_ERROR_STREAM_ONCE(...) if (this->isNodelet()) ROS_ERROR_STREAM_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_ERROR_COND(cond, ...) if (this->isNodelet()) ROS_ERROR_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_ERROR_COND(cond, __VA_ARGS__)
#define NODEWRAP_ERROR_STREAM_COND(cond, ...) if (this->isNodelet()) ROS_ERROR_STREAM_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_ERROR_THROTTLE(rate, ...) if (this->isNodelet()) ROS_ERROR_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_ERROR_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_ERROR_STREAM_THROTTLE(rate, ...) if (this->isNodelet()) ROS_ERROR_STREAM_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_ERROR_FILTER(filter, ...) if (this->isNodelet()) ROS_ERROR_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_ERROR_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_ERROR_STREAM_FILTER(filter, ...) if (this->isNodelet()) ROS_ERROR_STREAM_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_FATAL(...) if (this->isNodelet()) ROS_FATAL_NAMED(this->getName(), __VA_ARGS__); else ROS_FATAL(__VA_ARGS__)
#define NODEWRAP_FATAL_STREAM(...) if (this->isNodelet()) ROS_FATAL_STREAM_NAMED(this->getName(), __VA_ARGS__); else ROS_FATAL_STREAM(__VA_ARGS__)
#define NODEWRAP_FATAL_ONCE(...) if (this->isNodelet()) ROS_FATAL_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_FATAL_ONCE(__VA_ARGS__)
#define NODEWRAP_FATAL_STREAM_ONCE(...) if (this->isNodelet()) ROS_FATAL_STREAM_ONCE_NAMED(this->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_FATAL_COND(cond, ...) if (this->isNodelet()) ROS_FATAL_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_FATAL_COND(cond, __VA_ARGS__)
#define NODEWRAP_FATAL_STREAM_COND(cond, ...) if (this->isNodelet()) ROS_FATAL_STREAM_COND_NAMED(cond, this->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_FATAL_THROTTLE(rate, ...) if (this->isNodelet()) ROS_FATAL_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_FATAL_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_FATAL_STREAM_THROTTLE(rate, ...) if (this->isNodelet()) ROS_FATAL_STREAM_THROTTLE_NAMED(rate, this->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_FATAL_FILTER(filter, ...) if (this->isNodelet()) ROS_FATAL_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_FATAL_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_FATAL_STREAM_FILTER(filter, ...) if (this->isNodelet()) ROS_FATAL_STREAM_FILTER_NAMED(filter, this->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_MEMBER_DEBUG(...) if (this->getNode()->isNodelet()) ROS_DEBUG_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG(__VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_STREAM(...) if (this->getNode()->isNodelet()) ROS_DEBUG_STREAM_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM(__VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_ONCE(...) if (this->getNode()->isNodelet()) ROS_DEBUG_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_STREAM_ONCE(...) if (this->getNode()->isNodelet()) ROS_DEBUG_STREAM_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_DEBUG_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_STREAM_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_DEBUG_STREAM_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_DEBUG_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_STREAM_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_DEBUG_STREAM_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_DEBUG_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_MEMBER_DEBUG_STREAM_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_DEBUG_STREAM_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_DEBUG_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_MEMBER_INFO(...) if (this->getNode()->isNodelet()) ROS_INFO_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_INFO(__VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_STREAM(...) if (this->getNode()->isNodelet()) ROS_INFO_STREAM_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_STREAM(__VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_ONCE(...) if (this->getNode()->isNodelet()) ROS_INFO_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_STREAM_ONCE(...) if (this->getNode()->isNodelet()) ROS_INFO_STREAM_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_INFO_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_STREAM_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_INFO_STREAM_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_INFO_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_STREAM_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_INFO_STREAM_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_INFO_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_MEMBER_INFO_STREAM_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_INFO_STREAM_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_INFO_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_MEMBER_WARN(...) if (this->getNode()->isNodelet()) ROS_WARN_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_WARN(__VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_STREAM(...) if (this->getNode()->isNodelet()) ROS_WARN_STREAM_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_STREAM(__VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_ONCE(...) if (this->getNode()->isNodelet()) ROS_WARN_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_STREAM_ONCE(...) if (this->getNode()->isNodelet()) ROS_WARN_STREAM_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_WARN_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_STREAM_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_WARN_STREAM_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_WARN_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_STREAM_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_WARN_STREAM_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_WARN_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_MEMBER_WARN_STREAM_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_WARN_STREAM_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_WARN_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_MEMBER_ERROR(...) if (this->getNode()->isNodelet()) ROS_ERROR_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR(__VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_STREAM(...) if (this->getNode()->isNodelet()) ROS_ERROR_STREAM_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_STREAM(__VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_ONCE(...) if (this->getNode()->isNodelet()) ROS_ERROR_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_STREAM_ONCE(...) if (this->getNode()->isNodelet()) ROS_ERROR_STREAM_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_ERROR_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_STREAM_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_ERROR_STREAM_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_ERROR_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_STREAM_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_ERROR_STREAM_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_ERROR_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_MEMBER_ERROR_STREAM_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_ERROR_STREAM_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_ERROR_STREAM_FILTER(filter, __VA_ARGS__)

#define NODEWRAP_MEMBER_FATAL(...) if (this->getNode()->isNodelet()) ROS_FATAL_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL(__VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_STREAM(...) if (this->getNode()->isNodelet()) ROS_FATAL_STREAM_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_STREAM(__VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_ONCE(...) if (this->getNode()->isNodelet()) ROS_FATAL_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_STREAM_ONCE(...) if (this->getNode()->isNodelet()) ROS_FATAL_STREAM_ONCE_NAMED(this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_ONCE(__VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_FATAL_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_STREAM_COND(cond, ...) if (this->getNode()->isNodelet()) ROS_FATAL_STREAM_COND_NAMED(cond, this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_COND(cond, __VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_FATAL_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_STREAM_THROTTLE(rate, ...) if (this->getNode()->isNodelet()) ROS_FATAL_STREAM_THROTTLE_NAMED(rate, this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_THROTTLE(rate, __VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_FATAL_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_FILTER(filter, __VA_ARGS__)
#define NODEWRAP_MEMBER_FATAL_STREAM_FILTER(filter, ...) if (this->getNode()->isNodelet()) ROS_FATAL_STREAM_FILTER_NAMED(filter, this->getNode()->getName(), __VA_ARGS__); else ROS_FATAL_STREAM_FILTER(filter, __VA_ARGS__)

#endif
