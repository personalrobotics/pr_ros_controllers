#include <KdlTreeUtilities.h>

using namespace kdl_extension;

KdlTreeUtilities::KdlTreeUtilities()
{
}

KdlTreeUtilities::~KdlTreeUtilities()
{
}

void KdlTreeUtilities::getChainJointNames(const KDL::Chain& chain,
        std::vector<std::string>& jointNames) const
{
    jointNames.resize(chain.getNrOfJoints());
    const std::vector<KDL::Segment>&   segments = chain.segments;
    std::vector<std::string>::iterator nameIt   = jointNames.begin();
    for (unsigned int i = 0; i < segments.size(); ++i)
    {
        const KDL::Joint& joint = segments[i].getJoint();
        if (joint.getType() != KDL::Joint::None)
        {
            //is a joint
            *nameIt = joint.getName();
            ++nameIt;
        }
    }
}

void KdlTreeUtilities::getChainJointNames(const std::string& toolFrame,
        std::vector<std::string>& jointNames) const
{
    getChainJointNames(getBaseName(), toolFrame, jointNames);
}

void KdlTreeUtilities::getChainJointNames(const std::string& baseFrame, const std::string& toolFrame,
        std::vector<std::string>& jointNames) const
{
    KDL::Chain chain;
    if (!getChain(baseFrame, toolFrame, chain))
    {
        jointNames.clear();
        return;
    }
    getChainJointNames(chain, jointNames);
}

void KdlTreeUtilities::getJointNames(std::vector<std::string>& jointNames) const
{
    jointNames.resize(tree.getNrOfJoints());
    const KDL::SegmentMap& segments = tree.getSegments();
    for (KDL::SegmentMap::const_iterator segIt = segments.begin(); segIt != segments.end(); ++segIt)
    {
        const KDL::TreeElement& element = segIt->second;
        const KDL::Joint&       joint   = element.segment.getJoint();
        if (joint.getType() != KDL::Joint::None)
        {
            // is a joint
            jointNames[element.q_nr] = joint.getName();
        }
    }
}


bool KdlTreeUtilities::getChain(const std::string& baseFrame, const std::string& toolFrame, KDL::Chain &chain) const
{
    if (baseFrame == toolFrame)
    {
        chain = KDL::Chain();
        return false;
    }
    return (tree.getChain(baseFrame, toolFrame, chain));
}

