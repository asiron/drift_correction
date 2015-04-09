#ifndef DRIFT_CORRECTION_NODE_GUARD_H
#define DRIFT_CORRECTION_NODE_GUARD_H value

#include <ros/ros.h>

class DriftCorrectionNode
{
public:
  DriftCorrectionNode();
  ~DriftCorrectionNode();

private:
  ros::NodeHandle m_nh, m_privateNh;
  
  std::string m_globalFrameId;
  std::string m_torsoFrameId;
  std::string m_sensorFrameId;
  std::string m_odomFrameId;

  double m_publishingFrequency;

};

#endif