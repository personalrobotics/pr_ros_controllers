#ifndef KDL_TREE_ID_H
#define KDL_TREE_ID_H

#include "KdlChainIdRne.h"
#include "JointDynamicsData.h"
#include "KdlTreeUtilities.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <memory>

namespace kdl_extension {

class KdlTreeId : public KdlTreeUtilities
{
public:
    KdlTreeId();
    ~KdlTreeId();
    
    bool isBaseFrameInTree(const std::string& baseFrame);

    void getAccelInBaseFrame(KDL::Vector gravity, const std::string& gravityFrame, JointDynamicsData& jd, const std::string& base, KDL::Twist& a_in);
 
    bool treeRecursiveNewtonEuler(JointDynamicsData& jd, const std::string& baseFrame, const std::string& ignoreFrame, const KDL::Twist& v_in, const KDL::Twist& a_in, KDL::Wrench& f_out, KDL::RigidBodyInertia& I_out);

    void findChainFromNode(const std::string& baseFrame, const std::string& direction, std::string& toolFrame);

    int findBranchNodes(const std::string& baseFrame, const std::string& ignoreFrame, std::vector<std::string>& nodeList, std::vector<std::string>& direction);

    KDL::Wrench sumForces(const KDL::Wrenches& f_in);
 
protected:
    virtual void initialize();
    std::vector<std::string> jointNames;

    void setFrames(const std::string& gravityFrame_in, const std::string& baseFrame_in);

    std::string gravityFrame;
    std::string baseFrame;
    KDL::Chain gravBaseChain;
    std::auto_ptr<KDL::ChainFkSolverPos_recursive> chainFkSolver;
    KDL::JntArrayAcc joints;
};

}

#endif
