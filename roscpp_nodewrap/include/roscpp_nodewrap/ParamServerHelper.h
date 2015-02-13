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

/** \file ParamServerHelper.h
  * \brief Header file providing the ParamServerHelper class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_SERVER_HELPER_H
#define ROSCPP_NODEWRAP_PARAM_SERVER_HELPER_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS parameter service server helper
    * 
    * This class provides helper functions which are required to create
    * parameter service servers.
    */
  class ParamServerHelper {
  friend class ConfigServer;
  friend class ParamServer;
  public:
    /** \brief Constructor
      */ 
    ParamServerHelper();
    
    /** \brief Destructor
      */ 
    virtual ~ParamServerHelper();
    
  private:
    /** \brief Create a parameter service server (virtual declaration)
      */ 
    virtual ParamServer createServer(const AdvertiseParamOptions& options,
      const NodeImplPtr& nodeImpl) = 0;
  };
  
  /** \brief ROS parameter service server helper (templated version)
    */    
  template <class Spec> class ParamServerHelperT :
    public ParamServerHelper {
  public:
    /** \brief Constructor
      */ 
    ParamServerHelperT();
    
  private:
    /** \brief Create a parameter service server (implementation)
      */ 
    ParamServer createServer(const AdvertiseParamOptions& options, const
      NodeImplPtr& nodeImpl);
  };
};

#include <roscpp_nodewrap/ParamServerHelper.tpp>

#endif
