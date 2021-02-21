



#include "gmapping_wrapper.h"
#define MEASURE_TIME 

NAVIGATION::OccupancyGridMapConfig convert(const MapConfig& mapConfig) {
  NAVIGATION::OccupancyGridMapConfig config;
  config.sizeOfGridMap.width = (long)((mapConfig.xmax - mapConfig.xmin) / mapConfig.delta);
  config.sizeOfGridMap.height = (long)((mapConfig.ymax - mapConfig.ymin) / mapConfig.delta);
  config.sizeOfGrid.width = mapConfig.delta;
  config.sizeOfGrid.height = mapConfig.delta;
  config.globalPositionOfTopLeft.position.x = mapConfig.xmin;
  config.globalPositionOfTopLeft.position.y = mapConfig.ymax;
  config.globalPositionOfTopLeft.heading = 0;
  return config;
}

MapConfig convert(const NAVIGATION::OccupancyGridMapConfig& config) {
  MapConfig mapConfig;
  mapConfig.delta = config.sizeOfGrid.width;
  mapConfig.xmin = config.globalPositionOfTopLeft.position.x;
  mapConfig.xmax = config.sizeOfGridMap.width * config.sizeOfGrid.width + config.globalPositionOfTopLeft.position.x;
  mapConfig.ymin = -config.sizeOfGridMap.height * config.sizeOfGrid.height + config.globalPositionOfTopLeft.position.y;
  mapConfig.ymax = config.globalPositionOfTopLeft.position.y;
  return mapConfig;
}



GMapping::RangeSensor* getRangeSensorFromRangeData(const std::string& name, const RTC::RangeData& range) {
  return new GMapping::RangeSensor("FLASER", 
      range.ranges.length(), 
      range.config.angularRes, 
	    GMapping::OrientedPoint(
        range.geometry.geometry.pose.position.x, 
				range.geometry.geometry.pose.position.y, 
				range.geometry.geometry.pose.position.z),
	    0.0, 
      range.config.maxRange);
}


GMapping::GridSlamProcessor* getGridSlamProcessor(GMapping::RangeSensor *pRangeSensor, const MapConfig& mapConfig, const MotionModelParam& motionModelParam, 
  const UpdateParam& updateParam, const SamplerParam& sampleParam) {
  GMapping::GridSlamProcessor* pGridSlamProcessor = new GMapping::GridSlamProcessor();
	GMapping::SensorMap smap;
	smap.insert(make_pair(pRangeSensor->getName(), pRangeSensor));
	pGridSlamProcessor->setSensorMap(smap);

	pGridSlamProcessor->setMotionModelParameters(motionModelParam.srr, motionModelParam.srt, motionModelParam.str, motionModelParam.stt);
	pGridSlamProcessor->setUpdateDistances(updateParam.linearUpdate, updateParam.angularUpdate, updateParam.resampleThreshold);
	pGridSlamProcessor->setUpdatePeriod(updateParam.temporalUpdate);
	pGridSlamProcessor->setgenerateMap(false);

	GMapping::OrientedPoint initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
	pGridSlamProcessor->GridSlamProcessor::init(sampleParam.numParticles, mapConfig.xmin, mapConfig.ymin, mapConfig.xmax, mapConfig.ymax,
					mapConfig.delta, initialPose);
	pGridSlamProcessor->setllsamplerange(sampleParam.llsamplerange);
	pGridSlamProcessor->setllsamplestep(sampleParam.llsamplestep);

	pGridSlamProcessor->setminimumScore(sampleParam.minimumScore);

  return pGridSlamProcessor;
}

void setLaserToMatcher(GMapping::ScanMatcher& matcher, const RTC::RangeData& range, GMapping::RangeSensor* pRangeSensor) {
  double* laser_angles = new double[range.ranges.length()];
  double theta = range.config.minAngle;
  for(unsigned int i = 0; i < range.ranges.length();i++) {
    laser_angles[i]=theta;
	  theta += range.config.angularRes;
  }

  matcher.setLaserParameters(range.ranges.length(), laser_angles,
                             pRangeSensor->getPose());

  delete[] laser_angles;
  const double e = 0.1; // 0.1 mm
  matcher.setlaserMaxRange(range.config.maxRange);
  matcher.setusableRange(range.config.maxRange-e);
  matcher.setgenerateMap(true);
}


GMapping::RangeReading convertRange(const RTC::RangeData& rangeData, GMapping::RangeSensor *pRangeSensor, const RTC::TimedPose2D& odometry) {
  double* ranges_double = new double[rangeData.ranges.length()];
  for(unsigned int i = 0;i < rangeData.ranges.length();i++) {
    if(rangeData.ranges[i] < rangeData.config.minRange) {
      ranges_double[i] = rangeData.config.minRange;
    } else {
      ranges_double[i] = rangeData.ranges[i];
    }
  }
  GMapping::RangeReading reading(rangeData.ranges.length(),
                                 ranges_double,
                                 pRangeSensor,
                                 ((double)rangeData.tm.sec + ((double)rangeData.tm.nsec)/1000000000));
	delete[] ranges_double;
	reading.setPose(GMapping::OrientedPoint(
  	odometry.data.position.x,
		odometry.data.position.y,
		odometry.data.heading));
  return reading;
}

RTC::Pose2D convertPose(const GMapping::OrientedPoint& mpose) {
  RTC::Pose2D pose;
	pose.position.x = mpose.x;
	pose.position.y = mpose.y;
	pose.heading = mpose.theta;
  return pose;
}


// TODO: Use optional
bool updateOGMap(GMapping::GridSlamProcessor* pGridSlamProcessor, GMapping::RangeSensor *pRangeSensor, NAVIGATION::OccupancyGridMap& map, const RTC::RangeData& range, const double _occ_thresh) {

#ifdef MEASURE_TIME
  std::chrono::system_clock::time_point  start, end; // 型は auto で可
  start = std::chrono::system_clock::now(); // 計測開始時間
#endif


  GMapping::ScanMatcher matcher;
  setLaserToMatcher(matcher, range, pRangeSensor);
  MapConfig mapConfig = convert(map.config);

  GMapping::ScanMatcherMap smap({(mapConfig.xmin + mapConfig.xmax) / 2.0, (mapConfig.ymin + mapConfig.ymax) / 2.0}, mapConfig.xmin, mapConfig.ymin, mapConfig.xmax, mapConfig.ymax, mapConfig.delta);

  auto best = pGridSlamProcessor->getParticles()[pGridSlamProcessor->getBestParticleIndex()];
  for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent) {
    if(!n->reading) { 
      continue; // Reading is NULL 
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  if(map.config.sizeOfGridMap.width != (unsigned int) smap.getMapSizeX() || map.config.sizeOfGridMap.height != (unsigned int) smap.getMapSizeY()) {
    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    mapConfig.xmin = wmin.x; mapConfig.ymin = wmin.y;
    mapConfig.xmax = wmax.x; mapConfig.ymax = wmax.y;
    map.config = convert(mapConfig);
    map.cells.length(map.config.sizeOfGridMap.width * map.config.sizeOfGridMap.height);
  }

  for(int x = 0; x < smap.getMapSizeX(); x++) {
    for(int y = 0; y < smap.getMapSizeY(); y++) {
      double occ = smap.cell(GMapping::IntPoint(x,y));
      int index = (smap.getMapSizeY() - y - 1) * map.config.sizeOfGridMap.width + x;
      if(occ < 0) {
        map.cells[index] = NAVIGATION::MAP_BYTE_CELL_UNKNOWN_STATE; // previously 127;
      } else if(occ > _occ_thresh) {
        map.cells[index] = NAVIGATION::MAP_BYTE_CELL_OCCUPIED_STATE; // previously 255
      } else {
        map.cells[index] = NAVIGATION::MAP_BYTE_CELL_FREE_STATE; // prevoiusly 0
	    }
    }
  }
#ifdef MEASURE_TIME

// 処理
  end = std::chrono::system_clock::now();  // 計測終了時間
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); 
  std::cout << "[Mapper_gmapping] updateOGMap takes " << elapsed << " milliseconds." << std::endl;
#endif
  return true;
}
