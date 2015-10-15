#ifndef KDL_CHAIN_ID_RNE_H
#define KDL_CHAIN_ID_RNE_H

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>

namespace kdl_extension {

/**
 * \brief Recursive Newton Euler ID solver for chains in a tree.
 *
 * The algorithm implementation is based on the book "Rigid Body
 * Dynamics Algorithms" of Roy Featherstone, 2008
 * (ISBN:978-0-387-74314-1) See pages 96 for the pseudo-code.
 * 
 * It calculates the torques for the joints, given the motion of
 * the joints (q,qdot,qdotdot), external forces on the segments
 * (expressed in the segments reference frame) and the dynamical
 * parameters of the segments.  
 *
 * This function assumes that the contributions of the base joint
 * are calculated in the calling function.  
 */
class KdlChainIdRne {
    public:
        /**
         * Constructor for the solver, it will allocate all the necessary memory
         * \param chain The kinematic chain to calculate the inverse dynamics for, an internal copy will be made.
         * \param v_base The velocity of the base frame in world/parent frame
         * \param a_base The acceleration of the base frame due to prior joints and/or gravity, in world/parent frame.
         */
        KdlChainIdRne(const KDL::Chain& chain, const KDL::Twist& v_base, const KDL::Twist& a_base);
        ~KdlChainIdRne(){};
        
        /**
         * Function to calculate all necessary kinematic parameters. Must be 
         * called prior to DynamicsPass
         * Input parameters:
         * \param q The current joint positions
         * \param q_dot The current joint velocities
         * \param q_dotdot The current joint accelerations
         */
        int KinematicsPass(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, KDL::Twist& v_out, KDL::Twist& a_out);

        /**
         * Function to calculate from Cartesian forces to joint torques.  
         * Must call KinematicsPass prior to this function.
         * Input Parameters:
         * \param f_ext The external forces (no gravity) on the segments in that segments frame (different than Featherstone algorithm)
         * Output parameters:
         * \param torques the resulting torques for the joints
         * \param f_base  The wrench on the base frame for tree ID
         */
        int DynamicsPass(const KDL::Wrenches& f_ext,KDL::JntArray &torques, KDL::Wrench &f_base, std::vector<KDL::Wrench>& f, const KDL::RigidBodyInertia& I_ext, const KDL::RigidBodyInertia& I_seg, KDL::JntArray& Hv, KDL::RigidBodyInertia& I_base);

    private:
        KDL::Chain chain;
        unsigned int nj;
        unsigned int ns;
        std::vector<KDL::Frame> X;
        std::vector<KDL::Twist> S;
        std::vector<KDL::Twist> v;
        std::vector<KDL::Twist> a;
        std::vector<KDL::RigidBodyInertia> Ii;
        KDL::Wrench Fis;
        KDL::Twist vb;
        KDL::Twist ab;
};

}

#endif
