#include "drift_correction/drift_correction_node.h"

DriftCorrectionNode::DriftCorrectionNode() :
  m_nh(),
  m_privateNh("~"),
	m_globalFrameId("/world"),
	m_torsoFrameId("/torso"),
	m_sensorFrameId(""),
  m_trackedFrameId(""),
	m_odomFrameId("/odom"),
  m_publishingFrequency(100.0),
  m_tfListener(),
  m_tfBroadcaster(),
  m_currentDriftCorrection()

{
  m_privateNh.param("global_frame_id", m_globalFrameId, m_globalFrameId);
  m_privateNh.param("torso_frame_id", m_torsoFrameId, m_torsoFrameId);
  m_privateNh.param("odom_frame_id", m_odomFrameId, m_odomFrameId);
  m_privateNh.param("publishing_frequency", m_publishingFrequency, m_publishingFrequency);

  if (m_privateNh.hasParam("sensor_frame_id") == false)
  {
    ROS_FATAL("Sensor frame id is a required parameter");
    exit(-1);
  }
  else
  {
    m_privateNh.param("sensor_frame_id", m_sensorFrameId, m_sensorFrameId);
  }

  if (m_privateNh.hasParam("tracked_frame_id") == false)
  {
    ROS_FATAL("Tracked frame id is a required parameter");
    exit(-1);
  }
  else 
  {
    m_privateNh.param("tracked_frame_id", m_trackedFrameId, m_trackedFrameId);
  }

  ROS_INFO("Drift Correction Node initialized...");
}

DriftCorrectionNode::~DriftCorrectionNode()
{

}

void DriftCorrectionNode::run()
{
  m_publishingTimer = m_nh.createTimer(ros::Duration( 1 / m_publishingFrequency), &DriftCorrectionNode::publishTimerCallback, this);
  ros::spin();
}

void DriftCorrectionNode::publishTimerCallback(const ros::TimerEvent& event)
{

  if (m_tfListener.canTransform(m_globalFrameId, m_trackedFrameId, ros::Time(0)) == false)
  {
    ROS_INFO("Cannot transform from global frame: %s to tracked frame: %s", m_globalFrameId.c_str(), m_trackedFrameId.c_str());
    return;
  }

  if (m_tfListener.canTransform(m_odomFrameId, m_torsoFrameId, ros::Time(0)) == false)
  {
    ROS_INFO("Cannot transform from odometry frame: %s to torso frame: %s", m_odomFrameId.c_str(), m_torsoFrameId.c_str());
    return;
  }

  if (m_tfListener.canTransform(m_torsoFrameId, m_sensorFrameId, ros::Time(0)) == false)
  {
    ROS_INFO("Cannot transform from torso frame: %s to sensor frame: %s", m_torsoFrameId.c_str(), m_sensorFrameId.c_str());
    return;
  }

  tf::StampedTransform global_to_tracked;
  tf::StampedTransform odom_to_torso;
  tf::StampedTransform torso_to_sensor;
  try
  {
    m_tfListener.lookupTransform(m_globalFrameId, m_trackedFrameId, ros::Time(0), global_to_tracked);
    m_tfListener.lookupTransform(m_odomFrameId, m_torsoFrameId, ros::Time(0), odom_to_torso);
    m_tfListener.lookupTransform(m_torsoFrameId, m_sensorFrameId, ros::Time(0), torso_to_sensor);

    tf::Transform odom_to_sensor = odom_to_torso * torso_to_sensor;
    m_currentDriftCorrection = global_to_tracked * odom_to_sensor.inverse();
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  m_tfBroadcaster.sendTransform(tf::StampedTransform(m_currentDriftCorrection, ros::Time::now(), m_globalFrameId, m_odomFrameId));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drift_correction");
	DriftCorrectionNode dc;
  dc.run();

	return 0;
}