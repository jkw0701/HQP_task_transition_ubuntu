#ifndef __HQP__TASK__SCA__
#define __HQP__TASK__SCA__

#include "tasks/task-motion.h"
#include "trajectories/trajectory-base.h"
#include "constraint/constraint-equality.h"
#include "utils/motion.h"
#include "fcl/fcl_model.h"


namespace HQP
{
  namespace tasks
  {
    class TaskSelfCollision : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef constraint::ConstraintEquality ConstraintEquality;
      typedef fclmodel::FCL_MODEL FCL_MODEL;

 	    TaskSelfCollision(const std::string & name, RobotModel & robot, FCL_MODEL & model);
 
      int dim() const;
      const ConstraintBase & compute(const double t, Cref_vectorXd q, Cref_vectorXd v);

      const ConstraintBase & getConstraint() const;

      void setReference(TrajectorySample & ref);
      void getCenterInfo(Vector3d & cen_pos, Vector3d & cen_ori);
      void getClosestPointonMob(Vector3d & P_a);
      void getDistanceSCA(double &d);
      const TrajectorySample & getReference() const;

      const VectorXd & getDesiredAcceleration() const;
	    VectorXd getAcceleration(Cref_vectorXd dv) const;

      const VectorXd & position_error() const;
      const VectorXd & velocity_error() const;
      const VectorXd & position() const;
      const VectorXd & velocity() const;
      const VectorXd & position_ref() const;
      const VectorXd & velocity_ref() const;

      const double & Kp() const;
      const double & Kd() const;
      void Kp(double Kp);
      void Kd(double Kp);

      //Index frame_id() const;
	    void setBeta(const double & beta) { m_beta = beta; };
			void setPreviousSol(const VectorXd & old_sol) { m_old_sol = old_sol; }


    protected:
      FCL_MODEL & m_fclmodel;

	    MotionVector<double> m_p_error, m_v_error; // check -> Motion
      VectorXd m_p_error_vec, m_v_error_vec;
      VectorXd m_p, m_v;
	    VectorXd m_p_ref, m_v_ref_vec;
	    MotionVector<double> m_v_ref, m_a_ref;
      Transform3d m_M_ref, m_wMl;
	    double m_Kp;
	    double m_Kd;
	    VectorXd m_a_des;
	    MotionVector<double> m_drift;
      Matrix6x m_J;
      ConstraintEquality m_constraint;
      TrajectorySample m_ref;
      Vector3d m_center_pos;
      Vector3d m_center_ori;
      Vector3d m_P_a;
      double m_distance;

      	VectorXd m_old_sol;
			double m_beta;
    };    
  }
}

#endif // __HQP__TASK__OP_POSTURE__
