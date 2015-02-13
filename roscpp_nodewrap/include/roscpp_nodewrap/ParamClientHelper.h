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

/** \file ParamClientHelper.h
  * \brief Header file providing the ParamClientHelper class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_CLIENT_HELPER_H
#define ROSCPP_NODEWRAP_PARAM_CLIENT_HELPER_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS parameter service client helper
    * 
    * This class provides helper functions which are required to create
    * parameter service clients.
    */
  class ParamClientHelper {
  friend class ConfigClient;
  friend class ParamClient;
  public:
    /** \brief Constructor
      */ 
    ParamClientHelper();
    
    /** \brief Destructor
      */ 
    virtual ~ParamClientHelper();
    
  private:
    /** \brief Create a parameter service client (virtual declaration)
      */ 
    virtual ParamClient createClient(const ParamClientOptions& options,
      const NodeImplPtr& nodeImpl) = 0;
  };
  
  /** \brief ROS parameter service client helper (templated version)
    */    
  template <class Spec> class ParamClientHelperT :
    public ParamClientHelper {
  public:
    /** \brief Constructor
      */ 
    ParamClientHelperT();
    
  private:
    /** \brief Create a parameter service client (implementation)
      */ 
    ParamClient createClient(const ParamClientOptions& options, const
      NodeImplPtr& nodeImpl);
  };
};

#include <roscpp_nodewrap/ParamClientHelper.tpp>

#endif
