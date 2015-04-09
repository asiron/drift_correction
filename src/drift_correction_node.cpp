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
  m_privateNh.param("sensor_frame_id", m_sensorFrameId, m_sensorFrameId);
  m_privateNh.param("odom_frame_id", m_odomFrameId, m_odomFrameId);
  m_privateNh.param("publishing_frequency", m_publishingFrequency, m_publishingFrequency);

}

int main(int argc, char const *argv[])
{
	ros::init(argc, argv, "drift_correction")
	DriftCorrectionNode dc;


	return 0;
}