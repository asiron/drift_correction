#ifndef DRIFT_CORRECTION_NODE_GUARD_H
#define DRIFT_CORRECTION_NODE_GUARD_H value

#include <ros/ros.h>

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

  double m_publishingFrequency;

};

#endif