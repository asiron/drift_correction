#include "drift_correction/drift_correction_node.h"

DriftCorrectionNode::DriftCorrectionNode() :
  m_nh(),
  m_privateNh("~"),
	m_globalFrameId("/world"),
	m_torsoFrameId("/torso"),
	m_sensorFrameId(""),
	m_odomFrameId("/odom"),
  m_publishingFrequency(100.0)

{
  m_privateNh.param("global_frame_id", m_globalFrameId, m_globalFrameId);
  m_privateNh.param("torso_frame_id", m_torsoFrameId, m_torsoFrameId);
  m_privateNh.param("odom_frame_id", m_odomFrameId, m_odomFrameId);
  m_privateNh.param("publishing_frequency", m_publishingFrequency, m_publishingFrequency);

  if (m_privateNh.hasParam("sensor_frame_id") == false)
  {
    ROS_FATAL("Sensor frame id is required parameter");
    exit(-1);
  }
  else
  {
    m_privateNh.param("sensor_frame_id", m_sensorFrameId, m_sensorFrameId);
  }
}

DriftCorrectionNode::~DriftCorrectionNode()
{

}

void DriftCorrectionNode::run()
{
  m_publishingTimer = m_nh.createTimer(ros::Duration( 1 / m_publishingFrequency), &DriftCorrectionNode::publishTimerCallback, this);
}

void DriftCorrectionNode::publishTimerCallback(const ros::TimerEvent&)
{

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drift_correction");
	DriftCorrectionNode dc;
  dc.run();

	return 0;
}