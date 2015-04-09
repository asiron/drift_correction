#ifndef DRIFT_CORRECTION_NODE_GUARD_H
#define DRIFT_CORRECTION_NODE_GUARD_H value

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class DriftCorrectionNode
{
public:
  DriftCorrectionNode();
  ~DriftCorrectionNode();

  void run();
  void publishTimerCallback(const ros::TimerEvent&);

private:
  ros::NodeHandle m_nh, m_privateNh;

  ros::Timer m_publishingTimer;

  std::string m_globalFrameId;
  std::string m_torsoFrameId;
  std::string m_sensorFrameId;
  std::string m_odomFrameId;
  std::string m_trackedFrameId;

  double m_publishingFrequency;

  tf::TransformListener m_tfListener;
  tf::TransformBroadcaster m_tfBroadcaster;

  tf::Transform m_currentDriftCorrection;

};

#endif