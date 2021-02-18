// -*-C++-*-
/*!
 * @file  MapperSVC_impl.cpp
 * @brief Service implementation code of Mapper.idl
 *
 */

#include "MapperSVC_impl.h"
#include "Mapper_gmapping.h"

/*
 * Example implementational code for IDL interface NAVIGATION::OccupancyGridMapper
 */
NAVIGATION_OccupancyGridMapperSVC_impl::NAVIGATION_OccupancyGridMapperSVC_impl()
{
  // Please add extra constructor code here.
}


NAVIGATION_OccupancyGridMapperSVC_impl::~NAVIGATION_OccupancyGridMapperSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::initializeMap(const NAVIGATION::OccupancyGridMapConfig& config, const ::RTC::Pose2D& initialPose)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::initializeMap(const NAVIGATION::OccupancyGridMapConfig& config, RTC::Pose2D initialPose)>"
#endif
  return NAVIGATION::MAPPER_NG_NOTIMPL;
}

NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::startMapping()
{
  std::cout << "NAVIGATION_OccupanyGridMapperSVC_impl::startMapping() called" << std::endl;
	m_pRTC->startMap(true);
  return NAVIGATION::MAPPER_OK;
}

NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::stopMapping()
{
  std::cout << "NAVIGATION_OccupanyGridMapperSVC_impl::stopMapping() called" << std::endl;
	m_pRTC->startMap(false);
  return NAVIGATION::MAPPER_OK;
}

NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::suspendMapping()
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::suspendMapping()>"
#endif
  return NAVIGATION::MAPPER_NG_NOTIMPL;
}

NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::resumeMapping()
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::resumeMapping()>"
#endif
   return NAVIGATION::MAPPER_NG_NOTIMPL;
}

NAVIGATION::MAPPER_RETURN_VALUE NAVIGATION_OccupancyGridMapperSVC_impl::getState(NAVIGATION::MAPPER_STATE& state)
{
	if (m_pRTC->isMapStarted()) {
		state = NAVIGATION::MAPPER_MAPPING;
	} else {
		state = NAVIGATION::MAPPER_STOPPED;
	} 
  return NAVIGATION::MAPPER_OK;

}



// End of example implementational code



