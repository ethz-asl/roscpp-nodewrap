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

/** \file Managed.h
  * \brief Header file providing the Managed class interface
  */

#ifndef ROSCPP_NODEWRAP_MANAGED_H
#define ROSCPP_NODEWRAP_MANAGED_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Forward declaration of the templated manager
    */
  template <class T, typename I> class Manager;
  
  /** \brief Abstract basis of the ROS managed instance
    * 
    * This template class provides the abstract basis of a managed instance.
    */
  template <class T, typename I> class Managed {
  friend class Manager<T, I>;
  public:
    /** \brief Default constructor
      */
    Managed();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source managed instance which is being copied
      *   to this managed instance.
      */
    Managed(const Managed<T, I>& src);
    
    /** \brief Destructor
      */
    ~Managed();
    
    /** \brief Retrieve this managed instance's identifier
      */
    I getIdentifier() const;
    
    /** \brief Perform shutdown of the managed instance
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const Managed<T, I>& instance) const {
      return (impl < instance.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const Managed<T, I>& instance) const {
      return (impl == instance.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const Managed<T, I>& instance) const {
      return (impl != instance.impl);
    };
    
  protected:
    /** \brief Forward declaration of the manager implementation
      *   pointer type
      */
    typedef boost::shared_ptr<typename Manager<T, I>::Impl> ManagerImplPtr;
      
    /** \brief Forward declaration of the manager implementation weak
      *   pointer type
      */
    typedef boost::weak_ptr<typename Manager<T, I>::Impl> ManagerImplWPtr;
    
    /** \brief Abstract basis of the ROS managed instance implementation
      * 
      * This class provides the protected, abstract basis of the templated
      * managed instance implementation.
      */
    class Impl {
    friend class Manager<T, I>::Impl;
    public:
      /** \brief Constructor
        */
      Impl(const I& identifier, const ManagerImplPtr& manager);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the node owning this managed instance
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief Retrieve the identifier of this managed instance
        */
      const I& getIdentifier() const;
      
      /** \brief True, if this managed instance implementation is valid
        */
      virtual bool isValid() const = 0;
      
      /** \brief Perform shutdown of this managed instance implementation
        */
      virtual void shutdown() = 0;

      /** \brief Downcast this managed instance implementation
        *   (non-const version)
        */
      template <class D> D& as();
      
      /** \brief Downcast this managed instance implementation
        *   (const version)
        */
      template <class D> const D& as() const;
      
      /** \brief The identifier of this managed instance
        */ 
      I identifier;
      
      /** \brief The manager owning this managed instance
        */ 
      ManagerImplPtr manager;
    };

    /** \brief Declaration of the managed instance implementation pointer
      *   type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the managed instance implementation weak
      *   pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The managed instance's implementation
      */
    ImplPtr impl;
  };
};

#include <roscpp_nodewrap/Managed.tpp>

#endif
