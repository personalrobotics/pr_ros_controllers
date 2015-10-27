#ifndef JOINT_DYNAMICS_DATA_H
#define JOINT_DYNAMICS_DATA_H

#include <iostream>
#include <kdl/tree.hpp>
#include <kdl/chainidsolver.hpp>

namespace kdl_extension {

struct JntDynData
{
    double pos;
    double vel;
    double acc;
};

class JointDynamicsData
{
public:
    JointDynamicsData();
    ~JointDynamicsData();

    int PopulateJointInfo(const KDL::Chain& chain, KDL::JntArray& q, KDL::JntArray& q_dot, KDL::JntArray& q_dotdot);

    int PopulateExtForceInfo(const KDL::Chain& chain, KDL::Wrenches& f_ext);

    int PopulateExtForceInfo(const std::string& node, KDL::Wrenches& f_ext);

    /// @brief joint torque commands are the torques joints must apply to counteract forces
    int StoreJointTorqueCommands(const KDL::Chain& chain, const KDL::JntArray& tau);

    int StoreJointInertia(const KDL::Chain& chain, const KDL::JntArray& Hv);
    
    int StoreSegmentWrenches(const KDL::Chain& chain, const KDL::Wrenches& f_seg);

    void InitializeMaps(const KDL::Tree& tree);

    std::map<std::string, JntDynData> jointDataMap;
    std::map<std::string, KDL::Wrench> extForceMap, segForceMap;
    std::map<std::string, double> jointTorqueCommandMap;
    std::map<std::string, double> jointInertiaMap;
};

}

#endif
