// -*- C++ -*-
/*!
 * @file  Mapper_gmapping.h
 * @brief Mapper RTC using gmapping
 * @date  $Date$
 *
 * $Id$
 */

#ifndef MAPPER_GMAPPING_H
#define MAPPER_GMAPPING_H

#include "MobileRobotSVC_impl.h"
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "MobileRobotSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif
	 
#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"


using namespace RTC;

/*!
 * @class Mapper_gmapping
 * @brief Mapper RTC using gmapping
 *
 */
class Mapper_gmapping
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  Mapper_gmapping(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~Mapper_gmapping();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug
   * - DefaultValue: 0
   */
  int m_debug;
  /*!
   * The sigma used by the greedy endpoint matching
   * - Name: sigma sigma
   * - DefaultValue: 0.05
   */
  float m_sigma;
  /*!
   * The kernel in which to look for a correspondence
   * - Name: kernelSize kernelSize
   * - DefaultValue: 1
   */
  int m_kernelSize;
  /*!
   * The optimization step in translation
   * - Name: lstep lstep
   * - DefaultValue: 0.05
   */
  float m_lstep;
  /*!
   * The optimization step in rotation
   * - Name: astep astep
   * - DefaultValue: 0.05
   */
  float m_astep;
  /*!
   * The number of iterations of the scanmatcher
   * - Name: iterations iterations
   * - DefaultValue: 5
   */
  int m_iterations;
  /*!
   * The sigma of a beam used for likelihood computation
   * - Name: lsigma lsigma
   * - DefaultValue: 0.075
   */
  float m_lsigma;
  /*!
   * Gain to be used while evaluating the likelihood, for
   * smoothing the resampling effects
   * - Name: ogain ogain
   * - DefaultValue: 3.0
   */
  float m_ogain;
  /*!
   * Number of beams to skip in each scan.
   * - Name: lskip lskip
   * - DefaultValue: 0
   */
  int m_lskip;
  /*!
   * Minimum score for considering the outcome of the scan
   * matching good. Can avoid jumping pose estimates in large
   * open spaces when using laser scanners with limited range
   * (e.g. 5m). Scores go up to 600+, try 50 for example when
   * experiencing jumping estimate issues.
   * - Name: minimumScore minimumScore
   * - DefaultValue: 0.0
   */
  float m_minimumScore;
  /*!
   * Odometry error in translation as a function of translation
   * (rho/rho)
   * - Name: srr srr
   * - DefaultValue: 0.1
   */
  float m_srr;
  /*!
   * Odometry error in translation as a function of rotation
   * (rho/theta)
   * - Name: srt srt
   * - DefaultValue: 0.2
   */
  float m_srt;
  /*!
   * Odometry error in rotation as a function of translation
   * (theta/rho)
   * - Name: str str
   * - DefaultValue: 0.1
   */
  float m_str;
  /*!
   * Odometry error in rotation as a function of rotation
   * (theta/theta)
   * - Name: stt stt
   * - DefaultValue: 0.05
   */
  float m_stt;
  /*!
   * Process a scan each time the robot translates this far
   * - Name: linearUpdate linearUpdate
   * - DefaultValue: 1.0
   */
  float m_linearUpdate;
  /*!
   * Process a scan each time the robot rotates this far
   * - Name: angularUpdate angularUpdate
   * - DefaultValue: 0.5
   */
  float m_angularUpdate;
  /*!
   * Process a scan if the last scan proccessed is older than the
   * update time in seconds. A value less than zero will turn
   * time based updates off.
   * - Name: temporalUpdate temporalUpdate
   * - DefaultValue: -1.0
   */
  float m_temporalUpdate;
  /*!
   * The Neff based resampling threshold
   * - Name: resampleThreshold resampleThreshold
   * - DefaultValue: 0.5
   */
  float m_resampleThreshold;
  /*!
   * Number of particles in the filter
   * - Name: particles particles
   * - DefaultValue: 30
   */
  int m_particles;
  /*!
   * Initial map size
   * - Name: xmin xmin
   * - DefaultValue: -100
   */
  double m_xmin;
  /*!
   * Initial map size
   * - Name: ymin ymin
   * - DefaultValue: -100
   */
  double m_ymin;
  /*!
   * Initial map size
   * - Name: xmax xmax
   * - DefaultValue: 100
   */
  double m_xmax;
  /*!
   * Initial map size
   * - Name: ymax ymax
   * - DefaultValue: 100
   */
  double m_ymax;
  /*!
   * Processing parameters (resolution of the map)
   * - Name: delta delta
   * - DefaultValue: 0.05
   */
  double m_delta;
  /*!
   * Translational sampling range for the likelihood
   * - Name: llsamplerange llsamplerange
   * - DefaultValue: 0.01
   */
  float m_llsamplerange;
  /*!
   * Translational sampling step for the likelihood
   * - Name: llsamplestep llsamplestep
   * - DefaultValue: 0.01
   */
  float m_llsamplestep;
  /*!
   * Angular sampling range for the likelihood
   * - Name: lasamplerange lasamplerange
   * - DefaultValue: 0.005
   */
  float m_lasamplerange;
  /*!
   * Angular sampling step for the likelihood
   * - Name: lasamplestep lasamplestep
   * - DefaultValue: 0.005
   */
  float m_lasamplestep;
  /*!
   * How long (in seconds) between transform publications.
   * - Name: transform_publish_period transform_publish_period
   * - DefaultValue: 0.05
   */
  float m_transform_publish_period;
  /*!
   * Threshold on gmapping's occupancy values. Cells with greater
   * occupancy are considered occupied (i.e., set to 100 in the
   * resulting sensor_msgs/LaserScan).
   * - Name: occ_thresh occ_thresh
   * - DefaultValue: 0.25
   */
  float m_occ_thresh;
  /*!
   * Process 1 out of every this many scans (set it to a higher number to skip more scans)
   * - Name: throttle_scans throttle_scans
   * - DefaultValue: 1
   */
  int m_throttle_scans;
  /*!
   * How long (in seconds) between updates to the map. Lowering this number updates the occupancy grid more often, at the expense of greater computational load.
   * map_update_interval
   * - Name: map_update_interval
   * - DefaultValue: 5.0
   */
  float m_map_update_interval;


  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::RangeData m_range;
  /*!
   */
  InPort<RTC::RangeData> m_rangeIn;
  RTC::TimedPose2D m_odometry;
  /*!
   */
  InPort<RTC::TimedPose2D> m_odometryIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedPose2D m_estimatedPose;
  /*!
   */
  OutPort<RTC::TimedPose2D> m_estimatedPoseOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_gridMapperPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   */
  OGMapperSVC_impl m_mapper;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

	 GMapping::GridSlamProcessor* m_pGridSlamProcessor;
	 GMapping::RangeSensor *m_pRangeSensor;
	 GMapping::OdometrySensor *m_pOdometrySensor;

	 bool m_isScanReceived;
	 bool m_isOdomReceived;
	 bool m_isInit;

	 bool m_isMapStarted;

	 double m_lastScanTime;

 private:
 public:
	bool initMap(void);
	bool updateMap(void);
	bool updateOGMap(RTC::OGMap &map);

 public:
	RTC::OGMap m_map;

	void startMap(bool flag) {m_isMapStarted = flag;}
	bool isMapStarted() {return m_isMapStarted;}
};


extern "C"
{
  DLL_EXPORT void Mapper_gmappingInit(RTC::Manager* manager);
};

#endif // MAPPER_GMAPPING_H
