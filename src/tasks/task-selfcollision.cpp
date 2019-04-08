#include "tasks/task-selfcollision.h"
#include "utils/utils.h"

using namespace std;
namespace HQP
{
  namespace tasks
  {
    using namespace constraint;
    using namespace trajectories;

	TaskSelfCollision::TaskSelfCollision(const std::string & name, RobotModel & robot,HQP::fclmodel::FCL_MODEL & model):
      TaskMotion(name, robot),
      m_constraint(name, 1, robot.nv()),
      m_ref(12, 1),
      m_fclmodel(model)
    {
      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_p_error_vec.setZero(3);
      m_v_error_vec.setZero(3);
      m_p.resize(12);
      m_v.resize(6);
      m_p_ref.resize(12);
      m_v_ref_vec.resize(6);
      m_Kp = 0.0;
      m_Kd = 0.0;
      			m_beta = 1;
			m_old_sol.setZero(robot.nv());

      m_center_ori.setZero();
      m_center_pos.setZero();
      m_P_a.setZero();
      m_distance = 0.0;
    //   m_a_des.setZero(1);
    //   m_J.setZero(1, robot.nv());
    }

    int TaskSelfCollision::dim() const
    {
      return 1;
    }

    const double & TaskSelfCollision::Kp() const { return m_Kp; }

    const double & TaskSelfCollision::Kd() const { return m_Kd; }

    void TaskSelfCollision::Kp(double Kp)
    {
      assert(Kp.size()==1);
      m_Kp = Kp;
    }

    void TaskSelfCollision::Kd(double Kd)
    {
      assert(Kd.size()==1);
      m_Kd = Kd;
    }
		
    void TaskSelfCollision::setReference(TrajectorySample & ref)
    {
      m_ref = ref;
      m_M_ref.translation() = ref.pos.head<3>();
      typedef Eigen::Matrix<double, 3, 3> Matrix3;
      m_M_ref.linear()= Eigen::Map<const Matrix3>(&ref.pos(3), 3, 3);

      m_v_ref = MotionVector<double>(ref.vel);
      m_a_ref = MotionVector<double>(ref.acc);
    }

    const TrajectorySample & TaskSelfCollision::getReference() const
    {
      return m_ref;
    }

    const VectorXd & TaskSelfCollision::position_error() const
    {
      return m_p_error_vec;
    }

    const VectorXd & TaskSelfCollision::velocity_error() const
    {
      return m_v_error_vec;
    }

    const VectorXd & TaskSelfCollision::position() const
    {
      return m_p;
    }

    const VectorXd & TaskSelfCollision::velocity() const
    {
      return m_v;
    }

    const VectorXd & TaskSelfCollision::position_ref() const
    {
      return m_p_ref;
    }

    const VectorXd & TaskSelfCollision::velocity_ref() const
    {
      return m_v_ref_vec;
    }

    const VectorXd & TaskSelfCollision::getDesiredAcceleration() const
    {
      return m_a_des;
    }

	VectorXd TaskSelfCollision::getAcceleration(Cref_vectorXd dv) const
    {
      return m_constraint.matrix()*dv + m_drift.vector();
    }

    //Index TaskSelfCollision::frame_id() const
    //{
    //  return m_frame_id;
    //}

    const ConstraintBase & TaskSelfCollision::getConstraint() const
    {
      return m_constraint;
    }
    void TaskSelfCollision::getCenterInfo(Vector3d & cen_pos, Vector3d & cen_ori){
      m_center_pos = cen_pos;
      m_center_ori = cen_ori;
    }
    void TaskSelfCollision::getClosestPointonMob(Vector3d P_a){
      m_P_a = P_a;
    }
    void TaskSelfCollision::getDistanceSCA(double & d){
      m_distance = d;
    }



    const ConstraintBase & TaskSelfCollision::compute(const double t, Cref_vectorXd q, Cref_vectorXd v)
    {
 
      /////// Original Version 
      // VectorXd m_a_des_new;
      // MatrixXd m_J_new;

      // m_J_new.resize(1, dof);
      // m_J_new = m_fclmodel.getTaskVector().transpose() * m_fclmodel.getTaskJaco();
      // m_constraint.resize(1, m_robot.nv());
      
      // m_p_error = m_fclmodel.getTaskVector();
      // m_v_error = m_fclmodel.getTaskJaco() * m_robot.getRealJointVelocity();

      // m_a_des = -m_Kp.cwiseProduct(m_p_error.vector())
      //   - m_Kd.cwiseProduct(m_v_error.vector())


      // m_a_des_new = h_factor_dis(m_fclmodel.getClosestDistance(), 0.18, 0.05) * m_fclmodel.getTaskVector().transpose() * (m_Kp * m_fclmodel.getTaskVector() - m_Kd * m_fclmodel.getTaskJaco() * m_robot.getRealJointVelocity());

      // m_constraint.setMatrix(m_J_new);
      // m_constraint.setVector(m_a_des_new);

      // Vector3d u_vec, n_vec, avoid;
      // u_vec(0) = cos(m_center_ori(2));
      // u_vec(1) = sin(m_center_ori(2));
      // u_vec(2) = 0.0;

      // n_vec(0) = -sin(m_center_ori(2));
      // n_vec(1) =  cos(m_center_ori(2));
      // n_vec(2) = 0.0;

      // avoid = m_P_a - m_center_pos;

      // double v, w;
      // v = fabs(avoid.dot(u_vec));
      // w = fabs(avoid.dot(n_vec));

      // Matrix2d Mobile, Vector2d vel;
      // double r, b;
      // r = 0.165;
      // b = 0.5;
      // Mobile(0, 0) = r / 2.0;
      // Mobile(0, 1) = r / 2.0;
      // Mobile(1, 0) = r / (2.0 * b);
      // Mobile(1, 1) = -r / (2.0 * b);

      // vel(0) = v;
      // vel(1) = w;

      // VectorXd m_a_des_new;
      // MatrixXd m_J_new;

      // m_J_new.resize(2, dof + 2);
      // m_J_new.block(0, 0, 2, 2) = Mobile;
      // m_constraint.resize(2, m_robot.nv());

      // m_a_des_new = vel;

      // m_constraint.setMatrix(m_J_new);
      // m_constraint.setVector(m_a_des_new);

		
			// if (!transition_flag) {
			// 	m_constraint.setMatrix(m_fcl.getObsUnitVector3().transpose() * m_fcl.getObsJacobian3());
			// 	m_constraint.setVector(m_fcl.getObsUnitVector3().transpose() * m_fcl.getObsErrorVector3());

			// }
			// else {

			// 	m_constraint.setMatrix(m_fcl.getObsUnitVector3().transpose() * m_fcl.getObsJacobian3());
			// 	m_constraint.setVector(m_fcl.getObsUnitVector3().transpose() * m_fcl.getObsErrorVector3() +  (1.0 - m_fcl.getObsActivation3())* m_fcl.getObsUnitVector3().transpose() * m_fcl.getObsJacobian3() * m_old_sol);
			// //	cout << m_fcl.getObsUnitVector2().transpose() * m_fcl.getObsErrorVector2() << endl;
			// }


      return m_constraint;
      }    
  }
}
