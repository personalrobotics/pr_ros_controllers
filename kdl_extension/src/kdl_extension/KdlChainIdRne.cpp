#include <KdlChainIdRne.h>

using namespace KDL;
/***************************************************************************//**
 *
 * @brief Constructor for KdlChainIdRne (empty)
 * @param chain_
 * @param v_base
 * @param a_base
 *
 ******************************************************************************/
kdl_extension::KdlChainIdRne::KdlChainIdRne(const Chain& chain_, const Twist& v_base, const Twist& a_base):
    chain(chain_),nj(chain.getNrOfJoints()),ns(chain.getNrOfSegments()),
    X(ns),S(ns),v(ns),a(ns),Ii(ns),vb(v_base),ab(a_base)
{
}
/***************************************************************************//**
 *
 * @brief
 * @param q Position
 * @param q_dot Velocity
 * @param q_dotdot Acceleration
 * @param velOut
 * @param accelOut
 *
 ******************************************************************************/
int kdl_extension::KdlChainIdRne::KinematicsPass(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, Twist& velOut, Twist& accelOut)
{
    //Check sizes when in debug mode
    if(q.rows() != nj || q_dot.rows() != nj || q_dotdot.rows() != nj )
        return -1;

    unsigned int j = 0;

    //Sweep from root to leaf
    for (unsigned int i = 0; i < ns; i++) 
    {
        double q_,qdot_,qdotdot_;
        if (chain.getSegment(i).getJoint().getType()!=Joint::None) 
        {
            q_       = q(j);
            qdot_    = q_dot(j);
            qdotdot_ = q_dotdot(j);
            j++;
        } 
        else
        {
            q_ = qdot_ = qdotdot_ = 0.0;
        }

        //Calculate segment properties: X,S,vj,cj
        X[i] = chain.getSegment(i).pose(q_);//Remark this is the inverse of the
        //frame for transformations from
        //the parent to the current coord frame
        //Transform velocity and unit velocity to segment frame
        Twist vj = X[i].M.Inverse(chain.getSegment(i).twist(q_,qdot_));
        S[i]     = X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));
        //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
        //calculate velocity and acceleration of the segment (in segment coordinates)
        if (i == 0) 
        {
            v[i] = X[i].Inverse(vb) + vj;
            a[i] = X[i].Inverse(ab) + S[i]*qdotdot_ + v[i] * vj;
        } 
        else 
        {
            v[i] = X[i].Inverse(v[i-1]) + vj;
            a[i] = X[i].Inverse(a[i-1]) + S[i] * qdotdot_ + v[i] * vj;
        }
    }
    if (ns > 0)
    {
        velOut   = v[ns-1];
        accelOut = a[ns-1];
    }

    return 0;
}
/***************************************************************************//**
 *
 * @brief
 * @param torques
 * @param forceBase
 * @param inertiaExt
 * @param inertiaSeg
 * @param Hv
 * @param inertiaBase
 *
 ******************************************************************************/
int kdl_extension::KdlChainIdRne::DynamicsPass(const Wrenches& forceExt, JntArray &torques, Wrench& forceBase, std::vector<KDL::Wrench>& f, const KDL::RigidBodyInertia& inertiaExt, const KDL::RigidBodyInertia& inertiaSeg, KDL::JntArray &Hv, KDL::RigidBodyInertia& inertiaBase)
{
    if (torques.rows() != nj || forceExt.size()!=ns || Hv.rows() != nj || f.size() != ns || ns == 0)
    {
        return -1;
    }

    unsigned int j = 0;

    for (unsigned int i = 0; i < ns; i++)
    {
        //Calculate the force for the joint
        //Collect RigidBodyInertia and external forces
        Ii[i] = chain.getSegment(i).getInertia();
    }
    Ii[ns-1] = Ii[ns-1] + inertiaSeg;  // adding seg masses not compensated with other joints
    for (unsigned int i = 0; i < ns; i++)
    {
        f[i] = Ii[i] * a[i] + v[i] * (Ii[i] * v[i]) + forceExt[i];
    }

    Ii[ns-1] = Ii[ns-1] + inertiaExt; // for Hv calc only

    //Sweep from leaf to root
    j = nj-1;
    for (int i = ns-1; i >= 0; i--) 
    {

        if (i != 0) 
        {
            f[i-1]  = f[i-1]  + X[i] * f[i];
            Ii[i-1] = Ii[i-1] + X[i] * Ii[i];
        }

        Fis = Ii[i]*S[i];
        if (chain.getSegment(i).getJoint().getType() != Joint::None) 
        {
            Hv(j) = dot(S[i], Fis);
            torques(j--) = dot(S[i],f[i]);
        }
    }
    forceBase   = X[0]*f[0];
    inertiaBase = X[0]*Ii[0];

    return 0;
}
