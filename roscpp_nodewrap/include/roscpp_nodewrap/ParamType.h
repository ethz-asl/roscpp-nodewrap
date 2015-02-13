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

/** \file ParamType.h
  * \brief Header file providing the ParamType class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_TYPE_H
#define ROSCPP_NODEWRAP_PARAM_TYPE_H

#include <typeinfo>

#include <ros/ros.h>

namespace nodewrap {
  /** \brief ROS parameter type
    * 
    * This class provides information about a parameter's strong type.
    */
  class ParamType {
  public:
    /** \brief Default constructor
      */
    ParamType();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source parameter type which is being copied to
      *   this parameter type.
      */
    ParamType(const ParamType& src);

    /** \brief Destructor
      */
    ~ParamType();
    
    /** \brief Access the human-readable name of this type
      */
    std::string getName() const;
    
    /** \brief Query if this parameter type equals the specified template
     *    type
      */
    template <typename T> bool equals() const;
    
    /** \brief Templated accessor for the parameter type
      */
    template <typename T> static ParamType get();
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const ParamType& paramType) const {
      return (impl < paramType.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const ParamType& paramType) const {
      return (impl == paramType.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const ParamType& paramType) const {
      return (impl != paramType.impl);
    };
    
  private:
    /** \brief ROS parameter type implementation
      * 
      * This class provides the private implementation of the parameter type.
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::type_info& info);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Query if this parameter type is valid
        */
      bool isValid() const;
      
      /** \brief The human-readable name of this parameter type
        */
      std::string name;
    };
    
    /** \brief Declaration of the parameter type implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the parameter type implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief ROS parameter type implementation (templated version)
      * 
      * This class provides the private templated implementation of the
      * parameter type.
      */
    template <typename T> class ImplT :
      public Impl {
    public:
      /** \brief Constructor
        */
      ImplT();
      
      /** \brief Destructor
        */
      ~ImplT();
      
      /** \brief Access the singleton instance of this parameter type
        */
      static const ImplPtr& getInstance();
    };
    
    /** \brief The  parameter type's implementation
      */
    ImplPtr impl;    
  };
};

#include <roscpp_nodewrap/ParamType.tpp>

#endif
