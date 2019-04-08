#ifndef __HQP__TASK__JOINT_POSTURE__
#define __HQP__TASK__JOINT_POSTURE__

#include "tasks/task-motion.h"
#include "trajectories/trajectory-base.h"
#include "constraint/constraint-equality.h"
namespace HQP
{
  namespace tasks
  {
    class TaskJointPosture : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef trajectories::TrajectorySample TrajectorySample;
	  typedef constraint::ConstraintEquality ConstraintEquality;

      TaskJointPosture(const std::string & name, RobotModel & robot);

      int dim() const;

      const ConstraintBase & compute(const double t, Cref_vectorXd q, Cref_vectorXd v);
      const ConstraintBase & getConstraint() const;
	  const bool & getTransitionState() { return m_transition; };

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const VectorXd & getDesiredAcceleration() const;
	  VectorXd getAcceleration(Cref_vectorXd dv) const;

      const VectorXd & mask() const;
      void mask(const VectorXd & mask);

      const VectorXd & position_error() const;
      const VectorXd & velocity_error() const;
      const VectorXd & position() const;
      const VectorXd & velocity() const;
      const VectorXd & position_ref() const;
      const VectorXd & velocity_ref() const;

      const VectorXd & Kp();
      const VectorXd & Kd();
      void Kp(Cref_vectorXd Kp);
      void Kd(Cref_vectorXd Kp);

	  void setBeta(const double & beta) { m_beta = beta; };
	  void setOldSolution(Cref_vectorXd old_sol) { m_old_sol = old_sol;};
	  void setTransition(bool flag) { transition_flag = flag; };

    protected:
	  VectorXd m_Kp;
	  VectorXd m_Kd;
	  VectorXd m_p_error, m_v_error;
	  VectorXd m_p, m_v;
	  VectorXd m_a_des;
	  VectorXd m_mask;
      VectorXi m_activeAxes;
      TrajectorySample m_ref;
	  ConstraintEquality m_constraint;
	  
	  VectorXd m_old_sol;
	  double m_beta;
	  bool transition_flag;
    }; 
  }
}
#endif // __HQP__TASK__JOINT_POSTURE__
