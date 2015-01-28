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

/** \file Pluginlib.h
  * \brief Header file defining ROS pluginlib wrapper macros
  * 
  * The macros defined here wrap the pluginlib class export macros and
  * perform explicit template instantiation of the templated nodelet wrapper
  * around a node implementation.
  */

#ifndef ROSCPP_NODEWRAP_PLUGINLIB_H
#define ROSCPP_NODEWRAP_PLUGINLIB_H

#include <pluginlib/class_list_macros.h>

#ifdef PLUGINLIB_EXPORT_CLASS
#define NODEWRAP_EXPORT_CLASS(pkg, class_type) \
  template class nodewrap::Nodelet<class_type>; \
  PLUGINLIB_EXPORT_CLASS(nodewrap::Nodelet<class_type>, \
    nodelet::Nodelet);
#else
#define NODEWRAP_EXPORT_CLASS(pkg, class_type) \
  template class nodewrap::Nodelet<class_type>; \
  PLUGINLIB_DECLARE_CLASS(pkg, class_type, \
    nodewrap::Nodelet<class_type>, nodelet::Nodelet);
#endif

#endif
