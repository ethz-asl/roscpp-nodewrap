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

/** \file Forwards.h
  * \brief Header file providing forward declarations for the node wrapper
  */

#ifndef ROSCPP_NODEWRAP_FORWARDS_H
#define ROSCPP_NODEWRAP_FORWARDS_H

#include <ros/ros.h>

namespace roscpp_nodewrap {};
  
namespace nodewrap {
  using namespace roscpp_nodewrap;
  
  /** \brief Forward declaration of the node implementation
    */
  class NodeImpl;  
  /** \brief Forward declaration of the node implementation pointer type
    */
  typedef boost::shared_ptr<NodeImpl> NodeImplPtr;
  /** \brief Forward declaration of the node implementation weak pointer type
    */
  typedef boost::weak_ptr<NodeImpl> NodeImplWPtr;  
  
  /** \brief Forward declaration of the configuration service client
    */
  class ConfigClient;
  /** \brief Forward declaration of the configuration service client
    *   pointer type
    */
  typedef boost::shared_ptr<ConfigClient> ConfigClientPtr;
  /** \brief Forward declaration of the configuration service client weak
    *   pointer type
    */
  typedef boost::weak_ptr<ConfigClient> ConfigClientWPtr;
  
  /** \brief Forward declaration of the configuration service server
    */
  class ConfigServer;
  /** \brief Forward declaration of the configuration service server
    *   pointer type
    */
  typedef boost::shared_ptr<ConfigServer> ConfigServerPtr;
  /** \brief Forward declaration of the configuration service server weak
    *   pointer type
    */
  typedef boost::weak_ptr<ConfigServer> ConfigServerWPtr;
  
  /** \brief Forward declaration of the parameter service server
    */
  class ParamServer;
  /** \brief Forward declaration of the parameter service server
    *   pointer type
    */
  typedef boost::shared_ptr<ParamServer> ParamServerPtr;
  /** \brief Forward declaration of the parameter service server weak
    *   pointer type
    */
  typedef boost::weak_ptr<ParamServer> ParamServerWPtr;

  /** \brief Forward declaration of the parameter service server options
    */
  class ParamServerOptions;
  
  /** \brief Forward declaration of the parameter service client
    */
  class ParamClient;
  /** \brief Forward declaration of the parameter service client
    *   pointer type
    */
  typedef boost::shared_ptr<ParamClient> ParamClientPtr;
  /** \brief Forward declaration of the parameter service client weak
    *   pointer type
    */
  typedef boost::weak_ptr<ParamClient> ParamClientWPtr;

  /** \brief Forward declaration of the parameter service client options
    */
  class ParamClientOptions;
  
  /** \brief Forward declaration of the parameter service helper
    */
  class ParamServiceHelper;
  /** \brief Forward declaration of the parameter service helper
    *   pointer type
    */
  typedef boost::shared_ptr<ParamServiceHelper> ParamServiceHelperPtr;  
};

#endif
