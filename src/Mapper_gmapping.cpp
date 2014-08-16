// -*- C++ -*-
/*!
 * @file  Mapper_gmapping.cpp
 * @brief Mapper RTC using gmapping
 * @date $Date$
 *
 * $Id$
 */

#include "Mapper_gmapping.h"

// Module specification
// <rtc-template block="module_spec">
static const char* mapper_gmapping_spec[] =
  {
    "implementation_id", "Mapper_gmapping",
    "type_name",         "Mapper_gmapping",
    "description",       "Mapper RTC using gmapping",
    "version",           "1.0.0",
    "vendor",            "ssr",
    "category",          "Navigation",
    "activity_type",     "EVENTDRIVEN",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.start_map_update_in_activated", "false",
    "conf.default.x_min", "-10.0",
    "conf.default.x_max", "10.0",
    "conf.default.y_min", "-10.0",
    "conf.default.y_max", "10.0",
    "conf.default.resolution", "0.05",
    "conf.default.init_pose_x", "0.0",
    "conf.default.init_pose_y", "0.0",
    "conf.default.init_pose_th", "0.0",
    "conf.default.log_dir", "log_out",
    "conf.default.log_enable", "log_enable",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.start_map_update_in_activated", "spin",
    "conf.__widget__.x_min", "spin",
    "conf.__widget__.x_max", "spin",
    "conf.__widget__.y_min", "spin",
    "conf.__widget__.y_max", "spin",
    "conf.__widget__.resolution", "spin",
    "conf.__widget__.init_pose_x", "spin",
    "conf.__widget__.init_pose_y", "spin",
    "conf.__widget__.init_pose_th", "spin",
    "conf.__widget__.log_dir", "spin",
    "conf.__widget__.log_enable", "spin",
    // Constraints
    "conf.__constraints__.start_map_update_in_activated", "true,false",
    "conf.__constraints__.log_enable", "true,false",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Mapper_gmapping::Mapper_gmapping(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeIn("range", m_range),
    m_odometryIn("odometry", m_odometry),
    m_estimatedPoseOut("estimatedPose", m_estimatedPose),
    m_gridMapperPort("gridMapper")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Mapper_gmapping::~Mapper_gmapping()
{
}



RTC::ReturnCode_t Mapper_gmapping::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range", m_rangeIn);
  addInPort("odometry", m_odometryIn);
  
  // Set OutPort buffer
  addOutPort("estimatedPose", m_estimatedPoseOut);
  
  // Set service provider to Ports
  m_gridMapperPort.registerProvider("OGMapper", "RTC::OGMapper", m_mapper);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_gridMapperPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("start_map_update_in_activated", m_start_map_update_in_activated, "false");
  bindParameter("x_min", m_x_min, "-10.0");
  bindParameter("x_max", m_x_max, "10.0");
  bindParameter("y_min", m_y_min, "-10.0");
  bindParameter("y_max", m_y_max, "10.0");
  bindParameter("resolution", m_resolution, "0.05");
  bindParameter("init_pose_x", m_init_pose_x, "0.0");
  bindParameter("init_pose_y", m_init_pose_y, "0.0");
  bindParameter("init_pose_th", m_init_pose_th, "0.0");
  bindParameter("log_dir", m_log_dir, "log_out");
  bindParameter("log_enable", m_log_enable, "log_enable");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_gmapping::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmapping::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmapping::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Mapper_gmapping::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_gmapping::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_gmapping::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_gmapping::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmapping::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Mapper_gmapping::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_gmapping::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmapping::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void Mapper_gmappingInit(RTC::Manager* manager)
  {
    coil::Properties profile(mapper_gmapping_spec);
    manager->registerFactory(profile,
                             RTC::Create<Mapper_gmapping>,
                             RTC::Delete<Mapper_gmapping>);
  }
  
};


