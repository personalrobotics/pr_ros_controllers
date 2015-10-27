
#include <JointDynamicsData.h>

using namespace kdl_extension;

/***************************************************************************//**
 *
 * @brief Constructor for Joint Dynamics Data
 *
 ******************************************************************************/
JointDynamicsData::JointDynamicsData()
{
}

JointDynamicsData::~JointDynamicsData()
{
}
/***************************************************************************//**
 *
 * @brief Initialize maps and iterate through elements
 * @param tree Tree to retrieve segments from
 *
 ******************************************************************************/
void JointDynamicsData::InitializeMaps(const KDL::Tree& tree)
{
    // Initialize zeros in the right ways
    JntDynData  zeroData;
    KDL::Wrench zeroForce = KDL::Wrench::Zero();
    zeroData.pos          = zeroData.vel = zeroData.acc = 0.0;
    double      zeroTau   = 0.0;

    // Grab tree segment map and iterator
    KDL::SegmentMap                 segments = tree.getSegments();
    KDL::SegmentMap::const_iterator it       = segments.begin();

    // Iterate through map
    while (it != segments.end())
    {
        // Set all segment forces to zero
        extForceMap[it->first] = zeroForce;
        segForceMap[it->first] = zeroForce;

        // Find joints, and set data and torque to zero
        if (it->second.segment.getJoint().getType() != KDL::Joint::None)
        {
            jointDataMap[it->second.segment.getJoint().getName()] = zeroData;
            jointTorqueCommandMap[it->second.segment.getJoint().getName()] = zeroTau;
            jointInertiaMap[it->second.segment.getJoint().getName()] = zeroTau;
        }

        // Next element, please
        it++;
    }

    return;

}
/***************************************************************************//**
 *
 * @brief Match chain properties to jntArray properties and set joint data for each joints
 * @param chain Chain to compare to jntArray properties
 * @param q
 * @param q_dot
 * @param q_dotdot
 *
 ******************************************************************************/
int JointDynamicsData::PopulateJointInfo(const KDL::Chain& chain, KDL::JntArray& q, KDL::JntArray& q_dot, KDL::JntArray& q_dotdot)
{
    // Match chain properties to jntArray properties
    unsigned int ns = chain.getNrOfSegments();
    unsigned int nj = chain.getNrOfJoints();

    if (q.rows() != nj || q_dot.rows() != nj || q_dotdot.rows() != nj)
    {
        return -1;
    }

    // Iterate through all segments and joints in the chain
    unsigned int j = 0;
    for (unsigned int i = 0; i < ns; i++)
    {
        // Set joint data for each joint
        if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
        {
            const JntDynData& data = jointDataMap[chain.getSegment(i).getJoint().getName()];
            q(j) = data.pos;
            q_dot(j) = data.vel;
            q_dotdot(j) = data.acc;
            j++;
        }
    }

    return 0;

}
/***************************************************************************//**
 *
 * @brief Match chain properties to jntArray properties and set segment force map
 * @param chain Chain to compare to jntArray properties
 * @param forceExt
 *
 ******************************************************************************/
int JointDynamicsData::PopulateExtForceInfo(const KDL::Chain& chain, KDL::Wrenches& forceExt)
{
    unsigned int ns = chain.getNrOfSegments();
    std::map<std::string, KDL::Wrench>::const_iterator it;

    if (forceExt.size() != ns)
        return -1;

    for (unsigned int i = 0 ; i < ns; i++)
    {
        if ((it=extForceMap.find(chain.getSegment(i).getName())) != extForceMap.end())
        {
            forceExt[i] = it->second;
        }
    }

    return 0;
}
/***************************************************************************//**
 *
 * @brief Set segment force map at the specified node
 * @param node Node to set segment info at
 * @param forceExt
 *
 ******************************************************************************/
int JointDynamicsData::PopulateExtForceInfo(const std::string& node, KDL::Wrenches& forceExt)
{
    if (forceExt.size() != 1)
        return -1;

    std::map<std::string, KDL::Wrench>::const_iterator it;

    if ((it=extForceMap.find(node)) != extForceMap.end())
    {
        forceExt[0] = it->second;
    }

    return 0;
}
/***************************************************************************//**
 *
 * @brief Match chain properties to jntArray properties and set torque values at each joint
 * @param chain Chain to compare to jntArray properties
 * @param tau Calculated torque on a joint
 *
 ******************************************************************************/
int JointDynamicsData::StoreJointTorqueCommands(const KDL::Chain& chain, const KDL::JntArray& tau)
{
    // Match chain properties with jntArray properties
    unsigned int ns = chain.getNrOfSegments();
    unsigned int nj = chain.getNrOfJoints();

    if (tau.rows() != nj)
    {
        return -1;
    }

    // Iterate through joints
    unsigned int j = 0;
    for (unsigned int i = 0; i < ns; i++)
    {
        // Set torque value for each joint
        if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
        {
            // joint torque commands are opposite of calculated torque on a joint
            jointTorqueCommandMap[chain.getSegment(i).getJoint().getName()] = -tau(j);
            j++;
        }
    }

    return 0;
}
/***************************************************************************//**
 *
 * @brief Match chain properties to jntArray properties and set inertia at each joint
 * @param chain Chain to compare to jntArray properties
 * @param Hv
 *
 ******************************************************************************/
int JointDynamicsData::StoreJointInertia(const KDL::Chain& chain, const KDL::JntArray& Hv)
{
    // Match chain properties with jntArray properties
    unsigned int nj = chain.getNrOfJoints();
    unsigned int ns = chain.getNrOfSegments();

    if (Hv.rows() != nj)
    {
        return -1;
    }

    // Iterate through joints
    unsigned int j = 0;
    for (unsigned int i = 0; i < ns; i++)
    {
        // Set torque value for each joint
        if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
        {
            jointInertiaMap[chain.getSegment(i).getJoint().getName()] = Hv(j);
            j++;
        }
    }

    return 0;
}

int JointDynamicsData::StoreSegmentWrenches(const KDL::Chain& chain, const KDL::Wrenches& forceSeg)
{
    unsigned int ns = chain.getNrOfSegments();

    if (forceSeg.size() != ns)
    {
        return -1;
    }

    for (unsigned int i = 0; i < ns; i++)
    {
        segForceMap[chain.getSegment(i).getName()] = forceSeg[i];
    }

    return 0;
}
