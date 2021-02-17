// -*- C++ -*-
/*!
 * @file  Mapper_gmappingTest.cpp
 * @brief Mapper RTC using gmapping
 * @date $Date$
 *
 * $Id$
 */

#include "Mapper_gmappingTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* mapper_gmapping_spec[] =
  {
    "implementation_id", "Mapper_gmappingTest",
    "type_name",         "Mapper_gmappingTest",
    "description",       "Mapper RTC using gmapping",
    "version",           "1.1.0",
    "vendor",            "ssr",
    "category",          "Navigation",
    "activity_type",     "EVENTDRIVEN",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.sigma", "0.05",
    "conf.default.kernelSize", "1",
    "conf.default.lstep", "0.05",
    "conf.default.astep", "0.05",
    "conf.default.iterations", "5",
    "conf.default.lsigma", "0.075",
    "conf.default.ogain", "3.0",
    "conf.default.lskip", "0",
    "conf.default.minimumScore", "0.0",
    "conf.default.srr", "0.1",
    "conf.default.srt", "0.2",
    "conf.default.str", "0.1",
    "conf.default.stt", "0.05",
    "conf.default.linearUpdate", "1.0",
    "conf.default.angularUpdate", "0.5",
    "conf.default.temporalUpdate", "-1.0",
    "conf.default.resampleThreshold", "0.5",
    "conf.default.particles", "30",
    "conf.default.xmin", "-100",
    "conf.default.ymin", "-100",
    "conf.default.xmax", "100",
    "conf.default.ymax", "100",
    "conf.default.delta", "0.05",
    "conf.default.llsamplerange", "0.01",
    "conf.default.llsamplestep", "0.01",
    "conf.default.lasamplerange", "0.005",
    "conf.default.lasamplestep", "0.005",
    "conf.default.transform_publish_period", "0.05",
    "conf.default.occ_thresh", "0.25",
    "conf.default.throttle_scans", "1",
    "conf.default.map_update_interval", "5.0",

    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.sigma", "text",
    "conf.__widget__.kernelSize", "text",
    "conf.__widget__.lstep", "text",
    "conf.__widget__.astep", "text",
    "conf.__widget__.iterations", "text",
    "conf.__widget__.lsigma", "text",
    "conf.__widget__.ogain", "text",
    "conf.__widget__.lskip", "text",
    "conf.__widget__.minimumScore", "text",
    "conf.__widget__.srr", "text",
    "conf.__widget__.srt", "text",
    "conf.__widget__.str", "text",
    "conf.__widget__.stt", "text",
    "conf.__widget__.linearUpdate", "text",
    "conf.__widget__.angularUpdate", "text",
    "conf.__widget__.temporalUpdate", "text",
    "conf.__widget__.resampleThreshold", "text",
    "conf.__widget__.particles", "text",
    "conf.__widget__.xmin", "text",
    "conf.__widget__.ymin", "text",
    "conf.__widget__.xmax", "text",
    "conf.__widget__.ymax", "text",
    "conf.__widget__.delta", "text",
    "conf.__widget__.llsamplerange", "text",
    "conf.__widget__.llsamplestep", "text",
    "conf.__widget__.lasamplerange", "text",
    "conf.__widget__.lasamplestep", "text",
    "conf.__widget__.transform_publish_period", "text",
    "conf.__widget__.occ_thresh", "text",
    "conf.__widget__.throttle_scans", "text",
    "conf.__widget__.map_update_interval", "text",
    // Constraints

    "conf.__type__.debug", "int",
    "conf.__type__.sigma", "float",
    "conf.__type__.kernelSize", "int",
    "conf.__type__.lstep", "float",
    "conf.__type__.astep", "float",
    "conf.__type__.iterations", "int",
    "conf.__type__.lsigma", "float",
    "conf.__type__.ogain", "float",
    "conf.__type__.lskip", "int",
    "conf.__type__.minimumScore", "float",
    "conf.__type__.srr", "float",
    "conf.__type__.srt", "float",
    "conf.__type__.str", "float",
    "conf.__type__.stt", "float",
    "conf.__type__.linearUpdate", "float",
    "conf.__type__.angularUpdate", "float",
    "conf.__type__.temporalUpdate", "float",
    "conf.__type__.resampleThreshold", "float",
    "conf.__type__.particles", "int",
    "conf.__type__.xmin", "float",
    "conf.__type__.ymin", "float",
    "conf.__type__.xmax", "float",
    "conf.__type__.ymax", "float",
    "conf.__type__.delta", "float",
    "conf.__type__.llsamplerange", "float",
    "conf.__type__.llsamplestep", "float",
    "conf.__type__.lasamplerange", "float",
    "conf.__type__.lasamplestep", "float",
    "conf.__type__.transform_publish_period", "float",
    "conf.__type__.occ_thresh", "float",
    "conf.__type__.throttle_scans", "int",
    "conf.__type__.map_update_interval", "float",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Mapper_gmappingTest::Mapper_gmappingTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeIn("range", m_range),
    m_odometryIn("odometry", m_odometry),
    m_estimatedPoseOut("estimatedPose", m_estimatedPose),
    m_gridMapperPort("gridMapper"),
    m_mapServerClientPort("mapServerClient")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Mapper_gmappingTest::~Mapper_gmappingTest()
{
}



RTC::ReturnCode_t Mapper_gmappingTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("estimatedPose", m_estimatedPoseIn);

  // Set OutPort buffer
  addOutPort("range", m_rangeOut);
  addOutPort("odometry", m_odometryOut);

  // Set service provider to Ports
  m_mapServerClientPort.registerProvider("NAVIGATION_OccupancyGridMapServer", "NAVIGATION::OccupancyGridMapServer", m_NAVIGATION_OccupancyGridMapServer);

  // Set service consumers to Ports
  m_gridMapperPort.registerConsumer("OGMapper", "NAVIGATION::OccupancyGridMapper", m_mapper);

  // Set CORBA Service Ports
  addPort(m_gridMapperPort);
  addPort(m_mapServerClientPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("sigma", m_sigma, "0.05");
  bindParameter("kernelSize", m_kernelSize, "1");
  bindParameter("lstep", m_lstep, "0.05");
  bindParameter("astep", m_astep, "0.05");
  bindParameter("iterations", m_iterations, "5");
  bindParameter("lsigma", m_lsigma, "0.075");
  bindParameter("ogain", m_ogain, "3.0");
  bindParameter("lskip", m_lskip, "0");
  bindParameter("minimumScore", m_minimumScore, "0.0");
  bindParameter("srr", m_srr, "0.1");
  bindParameter("srt", m_srt, "0.2");
  bindParameter("str", m_str, "0.1");
  bindParameter("stt", m_stt, "0.05");
  bindParameter("linearUpdate", m_linearUpdate, "1.0");
  bindParameter("angularUpdate", m_angularUpdate, "0.5");
  bindParameter("temporalUpdate", m_temporalUpdate, "-1.0");
  bindParameter("resampleThreshold", m_resampleThreshold, "0.5");
  bindParameter("particles", m_particles, "30");
  bindParameter("xmin", m_xmin, "-100");
  bindParameter("ymin", m_ymin, "-100");
  bindParameter("xmax", m_xmax, "100");
  bindParameter("ymax", m_ymax, "100");
  bindParameter("delta", m_delta, "0.05");
  bindParameter("llsamplerange", m_llsamplerange, "0.01");
  bindParameter("llsamplestep", m_llsamplestep, "0.01");
  bindParameter("lasamplerange", m_lasamplerange, "0.005");
  bindParameter("lasamplestep", m_lasamplestep, "0.005");
  bindParameter("transform_publish_period", m_transform_publish_period, "0.05");
  bindParameter("occ_thresh", m_occ_thresh, "0.25");
  bindParameter("throttle_scans", m_throttle_scans, "1");
  bindParameter("map_update_interval", m_map_update_interval, "5.0");
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_gmappingTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmappingTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmappingTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Mapper_gmappingTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_gmappingTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_gmappingTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_gmappingTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmappingTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Mapper_gmappingTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_gmappingTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_gmappingTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void Mapper_gmappingTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(mapper_gmapping_spec);
    manager->registerFactory(profile,
                             RTC::Create<Mapper_gmappingTest>,
                             RTC::Delete<Mapper_gmappingTest>);
  }

};


