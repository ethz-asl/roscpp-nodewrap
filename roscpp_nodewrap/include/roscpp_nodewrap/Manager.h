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

/** \file Manager.h
  * \brief Header file providing the Manager class interface
  */

#ifndef ROSCPP_NODEWRAP_MANAGER_H
#define ROSCPP_NODEWRAP_MANAGER_H

#include <map>

#include <ros/ros.h>

#include <boost/enable_shared_from_this.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Forward declaration of the templated managed instance
    */
  template <class T, typename I> class Managed;
  
  /** \brief Abstract basis of the ROS manager
    * 
    * This template class provides the abstract basis of a manager of
    * instances.
    */
  template <class T, typename I> class Manager {
  friend class Managed<T, I>;
  public:
    /** \brief Default constructor
      */
    Manager();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source manager which is being copied to this
      *   task manager.
      */
    Manager(const Manager<T, I>& src);
    
    /** \brief Destructor
      */
    ~Manager();
    
    /** \brief Perform shutdown of the manager
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return impl ? (void*)1 : (void*)0;
    };
    
  protected:
    /** \brief Abstract basis of the ROS manager implementation
      * 
      * This class provides the protected, abstract basis of the templated
      * manager implementation.
      */
    class Impl :
      public boost::enable_shared_from_this<Impl> {
    friend class Managed<T, I>::Impl;
    public:
      /** \brief Default constructor
        */
      Impl();
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the node owning this manager
        */ 
      virtual const NodeImplPtr& getNode() const = 0;
      
      /** \brief Add a managed instance to this manager
        */
      void add(const typename T::ImplPtr& instance);
    
      /** \brief Remove a managed instance from this manager
        */
      typename T::ImplPtr remove(const I& identifier);
      
      /** \brief Find a managed instance of this manager
        */ 
      typename T::ImplPtr find(const I& identifier) const;
      
      /** \brief Perform shutdown of the manager implementation
        */
      virtual void shutdown() = 0;
            
      /** \brief Downcast this manager implementation (non-const version)
        */
      template <class D> D& as();
      
      /** \brief Downcast this manager implementation (const version)
        */
      template <class D> const D& as() const;
      
      /** \brief The manager's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The instances managed by this manager
        */ 
      std::map<I, typename T::ImplWPtr> instances;
    };
    
    /** \brief Declaration of the manager implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the manager implementation weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;    
    
    /** \brief The manager's implementation
      */
    ImplPtr impl;    
  };
};

#include <roscpp_nodewrap/Manager.tpp>

#endif
