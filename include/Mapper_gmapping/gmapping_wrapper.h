#pragma once


#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include "MapServerStub.h"
#include "MapperSkel.h"


#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

struct MapConfig {
  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double delta;
};

struct MotionModelParam {
  float srr;
  float srt;
  float str;
  float stt;
};

struct UpdateParam {
  float linearUpdate;
  float angularUpdate;
  float resampleThreshold;
  float temporalUpdate;
};

struct SamplerParam {
  int numParticles;
  float llsamplerange;
  float llsamplestep;
  float minimumScore;
};

MapConfig convert(const NAVIGATION::OccupancyGridMapConfig& config);

NAVIGATION::OccupancyGridMapConfig convert(const MapConfig& mapConfig); 

GMapping::RangeSensor* getRangeSensorFromRangeData(const std::string& name, const RTC::RangeData& range);

GMapping::GridSlamProcessor* getGridSlamProcessor(GMapping::RangeSensor *pRangeSensor, const MapConfig& mapConfig, const MotionModelParam& motionModelParam, const UpdateParam& updateParam, const SamplerParam& sampleParam);

void setLaserToMatcher(GMapping::ScanMatcher& matcher, const RTC::RangeData& range, GMapping::RangeSensor* pRangeSensor) ;

GMapping::RangeReading convertRange(const RTC::RangeData& rangeData, GMapping::RangeSensor *pRangeSensor, const RTC::TimedPose2D& odometry);


RTC::Pose2D convertPose(const GMapping::OrientedPoint& mpose);




bool updateOGMap(GMapping::GridSlamProcessor* pGridSlamProcessor, GMapping::RangeSensor *pRangeSensor, NAVIGATION::OccupancyGridMap &map, const RTC::RangeData& range, const double _occ_thresh);