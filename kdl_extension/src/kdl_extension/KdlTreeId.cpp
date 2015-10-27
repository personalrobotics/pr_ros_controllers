#include <KdlTreeId.h>
#include <iostream>

using namespace kdl_extension;

KdlTreeId::KdlTreeId()
{
}


KdlTreeId::~KdlTreeId()
{
}

bool KdlTreeId::isBaseFrameInTree(const std::string& baseFrame)
{
    return hasSegment(baseFrame);
}

void KdlTreeId::setFrames(const std::string& gravityFrame_in, const std::string& baseFrame_in)
{
    if (gravityFrame == gravityFrame_in && baseFrame == baseFrame_in)
    {
        // nothing to do
        return;
    }

    gravityFrame = gravityFrame_in;
    baseFrame    = baseFrame_in;
    getChain(gravityFrame, baseFrame, gravBaseChain);
    chainFkSolver.reset(new KDL::ChainFkSolverPos_recursive(gravBaseChain));
}

void KdlTreeId::getAccelInBaseFrame(KDL::Vector gravity, const std::string& gravityFrame, JointDynamicsData& jd, const std::string& base, KDL::Twist& accelIn)
{
    setFrames(gravityFrame, base);
    joints.resize(gravBaseChain.getNrOfJoints());
    jd.PopulateJointInfo(gravBaseChain, joints.q, joints.qdot, joints.qdotdot);
    KDL::Frame baseF;
    chainFkSolver->JntToCart(joints.q, baseF);

    gravity = baseF.M.Inverse() * gravity;
    accelIn = KDL::Twist(gravity, KDL::Vector::Zero());
}

bool KdlTreeId::treeRecursiveNewtonEuler(JointDynamicsData& jd, const std::string& baseFrame, const std::string& ignoreFrame, const KDL::Twist& velIn, const KDL::Twist& accelIn, KDL::Wrench& forceOut, KDL::RigidBodyInertia &inertiaIn)
{
    std::vector<std::string> nL, dir;
    std::string node;

    // If only one branch from base, find branch.  Otherwise, branch is
    // only the current base segment
    if (findBranchNodes(baseFrame, ignoreFrame, nL, dir) == 1)
    {
        findChainFromNode(nL[0], dir[0], node);
    }
    else
    {
        node = baseFrame;
    }

    // Create the chain from the branch found
    KDL::Chain chain;
    if (ignoreFrame == "null")
    {
        getChain(baseFrame, node, chain);
    }
    else
    {
        getChain(ignoreFrame, node, chain);
    }

    unsigned int nj = chain.getNrOfJoints();
    unsigned int ns = chain.getNrOfSegments();

    KDL::Twist velOut, accelOut;
    joints.resize(nj);
    KDL::JntArray tau(nj), Hv(nj);
    KDL::Wrenches forceExt(ns), forceSeg(ns);
    KDL::RigidBodyInertia inertiaExt, inertiaSum, inertiaSeg;

    KdlChainIdRne fs(chain, velIn, accelIn);
    jd.PopulateExtForceInfo(chain, forceExt);
    inertiaExt = inertiaSum = inertiaSeg = KDL::RigidBodyInertia::Zero();

    if (ns == 0)
    {
        forceExt.resize(1);
        jd.PopulateExtForceInfo(node, forceExt);
    }

    if (nj > 0)
    {
        // Using joint state data, populate the appropriate joint arrays
        // for the given chain
        jd.PopulateJointInfo(chain, joints.q, joints.qdot, joints.qdotdot);
    }

    // Run the kinematics pass of the rne algorithm, sending back tip
    // velocity and acceleration
    if (ns > 0)
    {
        fs.KinematicsPass(joints.q, joints.qdot, joints.qdotdot, velOut, accelOut);
    }
    else
    {
        velOut = velIn;
        accelOut = accelIn;
    }

    // Determine which branch to ignore
    std::string branchIgnore;
    if (ns == 1 && ignoreFrame == "null")
    {
        branchIgnore = baseFrame;
    }
    else if (ns <= 1)
    {
        branchIgnore = ignoreFrame;
    }
    else
    {
        branchIgnore = chain.getSegment(ns-2).getName();
    }

    // Find branches of current base node, call function recursively on
    // all its branches
    nL.clear();
    dir.clear();
    if (findBranchNodes(node, branchIgnore, nL, dir) != 0)
    {
        KDL::Wrenches forceBranches(nL.size());
        for (unsigned int i = 0; i<nL.size(); i++)
        {
            if (!treeRecursiveNewtonEuler(jd, nL[i], node, velOut, accelOut, forceBranches[i], inertiaExt))
            {
                inertiaSeg = inertiaSeg + inertiaExt; // inertia for next joint
            }
            else
            {
                inertiaSum = inertiaSum + inertiaExt; // inertia already compensated
            }
        }
        if (ns > 0)
        {
            forceExt[ns-1] += sumForces(forceBranches);
        }
        else
        {
            forceExt[0] += sumForces(forceBranches);
        }
    }

    // Call the dynamics half of the rne algorithm
    fs.DynamicsPass(forceExt, tau, forceOut, forceSeg, inertiaSum, inertiaSeg, Hv, inertiaIn);

    if (ns > 0)
    {
        jd.StoreSegmentWrenches(chain, forceSeg);
    }

    if (nj > 0)
    {
        // Store off the joint torque information
        jd.StoreJointTorqueCommands(chain, tau);
        jd.StoreJointInertia(chain, Hv);
        return true;
    }
    else
    {
        return false;
    }
}


void KdlTreeId::findChainFromNode(const std::string& baseFrame, const std::string& direction, std::string& toolFrame)
{
    toolFrame = baseFrame;
    KDL::SegmentMap::const_iterator it = tree.getSegment(toolFrame);

    if (direction == "parent")
    {
        while (it->second.children.size() < 2 && it != tree.getRootSegment())
        {
            toolFrame = it->second.parent->first;
            it = tree.getSegment(toolFrame);
        }
    } 
    else // direction == child
    {
        while (it->second.children.size() == 1)
        {
            toolFrame = it->second.children[0]->first;
            it        = tree.getSegment(toolFrame);
        }
    }

    return;
}


int KdlTreeId::findBranchNodes(const std::string& baseFrame, const std::string& ignoreFrame, std::vector<std::string>& nodeList, std::vector<std::string>& direction)
{
    const KDL::SegmentMap& segments = tree.getSegments();

    KDL::SegmentMap::const_iterator it = segments.find(baseFrame);
    if (it == segments.end()) {
      // TODO rt-safe
      std::cerr << "Could not find segment " << baseFrame << std::endl;
      throw std::runtime_error("Could not find segment");
    }

    if (it != tree.getRootSegment())
    {
        if (it->second.parent == segments.end()) {
          // TODO rt-safe
          std::cerr << "Could not find parent segment of " << baseFrame << std::endl;
          throw std::runtime_error("Could not find parent segment");
        }

        if (it->second.parent->first != ignoreFrame)
        {
            nodeList.push_back(it->second.parent->first);
            direction.push_back("parent");
        }
    }

    for (unsigned int i = 0; i < it->second.children.size(); i++)
    {
        if (it->second.children[i]->first != ignoreFrame)
        {
            nodeList.push_back(it->second.children[i]->first);
            direction.push_back("child");
        }
    }

    return nodeList.size();

}

KDL::Wrench KdlTreeId::sumForces(const KDL::Wrenches& forceIn)
{
    KDL::Wrench sum = KDL::Wrench::Zero();
    for (unsigned int i = 0; i < forceIn.size(); i++)
        sum += forceIn[i];

    return sum;
}

void KdlTreeId::initialize()
{
    getJointNames(jointNames);
}
