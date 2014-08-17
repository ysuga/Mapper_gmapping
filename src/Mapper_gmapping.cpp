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
    ""
  };
// </rtc-template>

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"


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

  m_mapper.setRTC(this);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_gridMapperPort);
  
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
  bindParameter("map_update_intefval", m_map_update_interval, "5.0");
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
	m_pGridSlamProcessor = new GMapping::GridSlamProcessor();
	m_isScanReceived = false;
	m_isOdomReceived = false;;
	m_isInit = false;
	m_isMapStarted = false;
	return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_gmapping::onDeactivated(RTC::UniqueId ec_id)
{
	delete m_pGridSlamProcessor;
	return RTC::RTC_OK;
}


bool Mapper_gmapping::initMap(void) {
	
	m_pRangeSensor = new GMapping::RangeSensor("LIDAR0", m_range.ranges.length(), m_range.config.angularRes, 
	GMapping::OrientedPoint(m_range.geometry.geometry.pose.position.x, 
							m_range.geometry.geometry.pose.position.y, 
							m_range.geometry.geometry.pose.position.z),
	0.0, m_range.config.maxRange);

	GMapping::OrientedPoint initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);

	GMapping::SensorMap smap;
	smap.insert(make_pair(m_pRangeSensor->getName(), m_pRangeSensor));


	m_pGridSlamProcessor->setSensorMap(smap);

	m_pGridSlamProcessor->setMotionModelParameters(m_srr, m_srt, m_str, m_stt);
	m_pGridSlamProcessor->setUpdateDistances(m_linearUpdate, m_angularUpdate, m_resampleThreshold);
	m_pGridSlamProcessor->setUpdatePeriod(m_temporalUpdate);
	m_pGridSlamProcessor->setgenerateMap(false);
	m_pGridSlamProcessor->GridSlamProcessor::init(m_particles, m_xmin, m_ymin, m_xmax, m_ymax,
					m_delta, initialPose);
	m_pGridSlamProcessor->setllsamplerange(m_llsamplerange);
	m_pGridSlamProcessor->setllsamplestep(m_llsamplestep);

	m_pGridSlamProcessor->setminimumScore(m_minimumScore);

	GMapping::sampleGaussian(1,time(NULL));

	// Initialize Map

	m_map.config.width = (long)((m_xmax - m_xmin) / m_delta);
	m_map.config.height = (long)((m_ymax - m_ymin) / m_delta);
	m_map.config.xScale = m_map.config.yScale = m_delta;
	m_map.config.origin.position.x = m_map.config.origin.position.y = m_map.config.origin.heading = 0;
	m_map.map.cells.length(m_map.config.width * m_map.config.height);

	return true;
}

bool Mapper_gmapping::updateMap(void) {
  GMapping::ScanMatcher matcher;
  double* laser_angles = new double[m_range.ranges.length()];
  double theta = m_range.config.minAngle;
  for(unsigned int i = 0; i < m_range.ranges.length();i++) {
    laser_angles[i]=theta;
	theta += m_range.config.angularRes;
  }

  matcher.setLaserParameters(m_range.ranges.length(), laser_angles,
                             m_pRangeSensor->getPose());

  delete[] laser_angles;
  matcher.setlaserMaxRange(m_range.config.maxRange);
  matcher.setusableRange(m_range.config.maxRange);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          m_pGridSlamProcessor->getParticles()[m_pGridSlamProcessor->getBestParticleIndex()];

  GMapping::Point center;
  center.x=(m_xmin + m_xmax) / 2.0;
  center.y=(m_ymin + m_ymax) / 2.0;

  GMapping::ScanMatcherMap smap(center, m_xmin, m_ymin, m_xmax, m_ymax, m_delta);

  for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent) {
    if(!n->reading) { continue; // Reading is NULL
	}
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  if(m_map.config.width != (unsigned int) smap.getMapSizeX() || m_map.config.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    m_xmin = wmin.x; m_ymin = wmin.y;
    m_xmax = wmax.x; m_ymax = wmax.y;

    m_map.config.width = smap.getMapSizeX();
    m_map.config.height = smap.getMapSizeY();
    m_map.config.origin.position.x = m_xmin;
    m_map.config.origin.position.y = m_ymin;
	m_map.map.cells.length(m_map.config.width * m_map.config.height);
  }

  for(int x=0; x < smap.getMapSizeX(); x++) {
    for(int y=0; y < smap.getMapSizeY(); y++) {
      GMapping::IntPoint p(x, y);
      double occ = smap.cell(p);
      if(occ < 0) {
		  m_map.map.cells[y * m_map.config.width + x] = -1;
	  } else if(occ > m_occ_thresh) {
		  m_map.map.cells[y * m_map.config.width + x] = 100;
      } else {
          m_map.map.cells[y * m_map.config.width + x] = 0;
	  }
    }
  }
  return true;
}


RTC::ReturnCode_t Mapper_gmapping::onExecute(RTC::UniqueId ec_id)
{
	if(m_rangeIn.isNew()) {
		m_rangeIn.read();
		double scanTime = m_range.tm.sec + ((double)m_range.tm.nsec)/1000000000;
		if(m_isInit && m_isMapStarted) {
			double* ranges_double = new double[m_range.ranges.length()];
			for(unsigned int i = 0;i < m_range.ranges.length();i++) {
				if(m_range.ranges[i] < m_range.config.minRange) {
					ranges_double[i] = m_range.config.minRange;
				} else {
					ranges_double[i] = m_range.ranges[i];
				}
			}
			
			GMapping::RangeReading reading(m_range.ranges.length(),
                                 ranges_double,
                                 m_pRangeSensor,
                                 ((double)m_range.tm.sec + ((double)m_range.tm.nsec)/1000000000));
			delete[] ranges_double;

			GMapping::OrientedPoint gmap_pose = GMapping::OrientedPoint(
				m_odometry.data.position.x,
				m_odometry.data.position.y,
				m_odometry.data.heading);

			reading.setPose(gmap_pose);

			if(!m_pGridSlamProcessor->processScan(reading)) {
				std::cerr << "Error: processScan Failed." << std::endl;
			} else {
				GMapping::OrientedPoint mpose = m_pGridSlamProcessor->getParticles()[m_pGridSlamProcessor->getBestParticleIndex()].pose;
				m_estimatedPose.data.position.x = mpose.x;
				m_estimatedPose.data.position.y = mpose.y;
				m_estimatedPose.data.heading = mpose.theta;
				setTimestamp<RTC::TimedPose2D>(m_estimatedPose);
				m_estimatedPoseOut.write();

				double interval = scanTime - m_lastScanTime;
				if(this->m_map_update_interval < interval) {
					if(!updateMap()) {
						std::cerr << "Update Map Failed." << std::endl;
					}
					m_lastScanTime = scanTime;
				}
			}

		}

		m_isScanReceived = true;
	}

	if(m_odometryIn.isNew()) {
		m_odometryIn.read();
		m_isOdomReceived = true;
	}

	if(m_isOdomReceived && m_isScanReceived) {
		if (!m_isInit) {
			m_isInit = initMap();
			m_lastScanTime =  m_range.tm.sec + ((double)m_range.tm.nsec)/1000000000;
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


