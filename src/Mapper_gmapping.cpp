// -*- C++ -*-
/*!
 * @file  Mapper_gmapping.cpp
 * @brief Mapper RTC using gmapping
 * @date $Date$
 *
 * $Id$
 */

#include <thread>
#include <chrono>

#include "Mapper_gmapping.h"

#include "gmapping_wrapper.h"

// #define MEASURE_TIME 

// Module specification
// <rtc-template block="module_spec">
static const char* mapper_gmapping_spec[] =
  {
    "implementation_id", "Mapper_gmapping",
    "type_name",         "Mapper_gmapping",
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
    "conf.default.xmin", "-12",
    "conf.default.ymin", "-12",
    "conf.default.xmax", "12",
    "conf.default.ymax", "12",
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
Mapper_gmapping::Mapper_gmapping(RTC::Manager* manager)
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
  m_gridMapperPort.registerProvider("NAVIGATION_OccupancyGridMapper", "NAVIGATION::OccupancyGridMapper", m_mapper);

  // Set service consumers to Ports
  m_mapServerClientPort.registerConsumer("NAVIGATION_OccupancyGridMapServer", "NAVIGATION::OccupancyGridMapServer", m_NAVIGATION_OccupancyGridMapServer);

  m_mapper.setRTC(this);
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
  std::cout << "[Mapper_gmapping] onActivated called." << std::endl;
  MapConfig mapConfig;
  mapConfig.xmin = m_xmin;
  mapConfig.xmax = m_xmax;
  mapConfig.ymin = m_ymin;
  mapConfig.ymax = m_ymax;
  mapConfig.delta = m_delta;

  MotionModelParam motion;
  motion.srr = m_srr;
  motion.srt = m_srt;
  motion.str = m_str;
  motion.stt = m_stt;

  UpdateParam updateParam;
  updateParam.linearUpdate = m_linearUpdate;
  updateParam.angularUpdate = m_angularUpdate;
  updateParam.temporalUpdate = m_temporalUpdate;
  updateParam.resampleThreshold = m_resampleThreshold;

  SamplerParam sampleParam;
  sampleParam.numParticles = m_particles;
  sampleParam.llsamplerange = m_llsamplerange;
  sampleParam.llsamplestep = m_llsamplestep;
  sampleParam.minimumScore = m_minimumScore;
  
	m_isMapStarted = false;
  m_isMapStopping = false;

  bool waitingRange = true;
  bool waitingPose = true;
  while(waitingRange || waitingPose) {
    waitingRange = !m_rangeIn.isNew();
    waitingPose  = !m_odometryIn.isNew();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (waitingRange) std::cout << "[Mapper_gmapping] Waiting for RangeData is arrived." << std::endl;
    if (waitingPose) std::cout << "[Mapper_gmapping] Waiting for Odometry (TimedPose2D) data is arrived." << std::endl;
  }
  m_rangeIn.read();
  m_odometryIn.read();
	m_pRangeSensor = getRangeSensorFromRangeData("FLASER", m_range);
  m_pGridSlamProcessor = getGridSlamProcessor(m_pRangeSensor, mapConfig, motion, updateParam, sampleParam);
	//GMapping::sampleGaussian(1,time(NULL));

	m_lastScanTime =  m_range.tm.sec + ((double)m_range.tm.nsec)/1000000000;
	// Initialize Map
  m_map.config = convert(mapConfig);
	m_map.cells.length(m_map.config.sizeOfGridMap.width * m_map.config.sizeOfGridMap.height);
  m_isMapInitialized = false;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_gmapping::onDeactivated(RTC::UniqueId ec_id)
{
	delete m_pGridSlamProcessor;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Mapper_gmapping::onExecute(RTC::UniqueId ec_id)
{
  if (m_isMapStarted && m_isMapStopping) {
    m_isMapStarted = false;
  }


	if(m_odometryIn.isNew()) {
		m_odometryIn.read();
    if (!m_isMapStarted) {
      m_estimatedPose = m_odometry;
      m_estimatedPoseOut.write();
    }
	}

  if(m_rangeIn.isNew()) {
		m_rangeIn.read();
		const double scanTime = m_range.tm.sec + ((double)m_range.tm.nsec)/1000000000;
    //std::cout << "scan/last=" << scanTime << "/" << m_lastScanTime << std::endl;
    if (m_isMapStarted) {
      bool scanResult = false;
      {
#ifdef MEASURE_TIME
        std::cout << "[Mapper_gmapping] processingScan ...." << std::endl;
        std::chrono::system_clock::time_point  start, end; // 型は auto で可
        start = std::chrono::system_clock::now(); // 計測開始時間
#endif
        scanResult = m_pGridSlamProcessor->processScan(convertRange(m_range, m_pRangeSensor, m_odometry));
#ifdef MEASURE_TIME
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); 
        std::cout << "[Mapper_gmapping] processScan takes " << elapsed << " milliseconds." << std::endl;
#endif
      }
      if (scanResult) {
        if(!m_isMapInitialized || (this->m_map_update_interval < (scanTime - m_lastScanTime))) {
          std::cout << "[Mapper_gmapping] Updating Map...." << std::endl;
          auto ret = updateOGMap(m_pGridSlamProcessor, m_pRangeSensor, m_map, m_range, m_occ_thresh);
          std::cout << "[Mapper_gmapping] Update Success." << std::endl;
          {
#ifdef MEASURE_TIME
            std::chrono::system_clock::time_point  start, end; // 型は auto で可
            start = std::chrono::system_clock::now(); // 計測開始時間
#endif
            m_NAVIGATION_OccupancyGridMapServer->updateWholeMap(m_map);
#ifdef MEASURE_TIME
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); 
            std::cout << "[Mapper_gmapping] OccupancyGridMapServer::updateWholeMap takes " << elapsed << " milliseconds." << std::endl;
#endif
          }
          m_lastScanTime = scanTime;
          m_isMapInitialized = true;
        }
      }
      m_estimatedPose.data = convertPose(m_pGridSlamProcessor->getParticles()[m_pGridSlamProcessor->getBestParticleIndex()].pose);
      setTimestamp<RTC::TimedPose2D>(m_estimatedPose);
      m_estimatedPoseOut.write();
    }
  }
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


