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

/** \file ParamCallbackHelper.h
  * \brief Header file providing the ParamCallbackHelper class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_CALLBACK_HELPER_H
#define ROSCPP_NODEWRAP_PARAM_CALLBACK_HELPER_H

#include <XmlRpcValue.h>

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace nodewrap {
  /** \brief ROS parameter callback helper parameters
    */
  
  class ParamCallbackHelperCallParams {
  public:
    boost::shared_ptr<std::string> key;
    boost::shared_ptr<XmlRpc::XmlRpcValue> value;
  };

  /** \brief ROS parameter-specific parameters
    */
  
  template <typename T> class ParamSpecCallParams {
  public:
    boost::shared_ptr<std::string> key;
    boost::shared_ptr<T> value;
  };
  
  /** \brief ROS parameter event
    */
  template <typename T> class ParameterEvent {
  public:
    typedef std::string KeyType;
    typedef T ValueType;
    typedef boost::shared_ptr<KeyType> KeyPtr;
    typedef boost::shared_ptr<ValueType> ValuePtr;
    typedef boost::function<bool(ParameterEvent<ValueType>&)>
      CallbackType;

    static void call(const CallbackType& callback,
        ParamSpecCallParams<ValueType>& params) {
      ParameterEvent<ValueType> event(params.key, params.value);
      callback(event);
    };

    ParameterEvent(const KeyPtr& key, const ValuePtr& value) :
      key(key),
      value(value) {
    };

    /** \brief Returns a const-reference to the request
      */
    const KeyType& getKey() const {
      return *key;
    };
    
    /** \brief Returns a non-const reference to the response
      */
    const ValueType& getValue() const {
      return *value;
    };
  private:
    KeyPtr key;
    ValuePtr value;
  };
  
  template <typename T> class ParameterSpec {
  public:
    typedef std::string KeyType;
    typedef T ValueType;
    typedef boost::shared_ptr<KeyType> KeyPtr;
    typedef boost::shared_ptr<ValueType> ValuePtr;
    typedef boost::function<void(const ValueType&)> CallbackType;

    static bool call(const CallbackType& callback,
        ParamSpecCallParams<ValueType>& params) {
      return callback(*params.key, *params.value);
    };
  };
  
  /** \brief ROS parameter callback helper
    */
  
  class ParamCallbackHelper {
  public:
    virtual ~ParamCallbackHelper() {
    };
    
    virtual void call(ParamCallbackHelperCallParams& params) = 0;
  };
  
  typedef boost::shared_ptr<ParamCallbackHelper> ParamCallbackHelperPtr;
  
  /** \brief ROS parameter callback helper implementation
    */  
  
  template <typename Spec> class ParamCallbackHelperT :
    public ParamCallbackHelper {
  public:
    typedef typename Spec::KeyType KeyType;
    typedef typename Spec::ValueType ValueType;
    typedef typename Spec::KeyPtr KeyPtr;
    typedef typename Spec::ValuePtr ValuePtr;
    typedef typename Spec::CallbackType Callback;
    typedef boost::function<ValuePtr()> ValueCreateFunction;
    
    typedef typename boost::add_const<ValueType>::type ConstType;
    typedef typename boost::remove_const<ValueType>::type NonConstType;
    
//     ParamCallbackHelperT(const Callback& callback, const
//         ValueCreateFunction& createValue = static_cast<ValuePtr(*)()>(
//         defaultValueCreateFunction<ValueType>)) :
//       callback(callback),
//       createValue(createValue) {
//     };
// 
//     void call(ParamCallbackHelperCallParams& params) {
//       namespace ser = serialization;
//       RequestPtr req(create_req_());
//       ResponsePtr res(create_res_());
// 
//       ser::deserializeMessage(params.request, *req);
// 
//       ParamSpecCallParams<ValueType> callParams;
//       call_params.request = req;
//       call_params.response = res;
//       call_params.connection_header = params.connection_header;
//       
//       Spec::call(callback, callParams);
//       params.response = ser::serializeServiceResponse(ok, *res);
//     };

  private:
    Callback callback;
    ValueCreateFunction createValue;
  };
};

#endif
