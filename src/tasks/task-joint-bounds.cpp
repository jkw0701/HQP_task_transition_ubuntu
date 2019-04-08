#include "tasks/task-joint-bounds.h"
namespace HQP
{
   namespace tasks
  {
	  using namespace constraint;
	  using namespace trajectories;

	  TaskJointLimit::TaskJointLimit(const std::string & name, RobotModel & robot) 
		  : TaskBase(name, robot), m_constraint(name, robot.nv(), robot.nv()) {
		  m_robot = robot;
		  m_velocity_limit_ = 2000.0;
		  if (robot.type() == 0) {
			  m_constraint.setLowerBound(-m_velocity_limit_ * VectorXd(robot.nv()).setOnes());
			  m_constraint.setUpperBound(m_velocity_limit_ * VectorXd(robot.nv()).setOnes());
			  m_Kp.setZero(robot.nv());
			  m_Kd.setZero(robot.nv());
			  VectorXd m = VectorXd::Ones(robot.nv());
			  mask(m);
		  }
		  else if (robot.type() == 1) {
			  m_constraint.setLowerBound(-m_velocity_limit_ * VectorXd(robot.nv()).setOnes());
			  m_constraint.setUpperBound(m_velocity_limit_ * VectorXd(robot.nv()).setOnes());
			  m_Kp.setZero(robot.nv());
			  m_Kd.setZero(robot.nv());
			  VectorXd m = VectorXd::Ones(robot.nv());
			  mask(m);
		  }
		  m_buffer = 10.0 * M_PI / 180.0;
	  }

	  int TaskJointLimit::dim() const
	  {
		  return m_robot.nv();
	  }

	  const ConstraintBase & TaskJointLimit::compute(const double t, Cref_vectorXd q, Cref_vectorXd v) {
		  if (m_robot.type() == 0) {
			  for (int i = 0; i < m_robot.nv(); i++) {
				  if (q(i) < m_q_lbound(i) + m_buffer) {
					  m_constraint.lowerBound()(i) = m_Kp(i) * ((m_q_lbound(i) + m_buffer) - q(i)) - m_Kd(i) * v(i);
					  m_constraint.upperBound()(i) = m_velocity_limit_;

					  if (m_constraint.lowerBound()(i) > m_velocity_limit_)
						  m_constraint.lowerBound()(i) = m_velocity_limit_;

					  m_constraint.upperBound()(i) = m_velocity_limit_;
				  }
				  else if (q(i) > m_q_ubound(i) - m_buffer) {
					  m_constraint.upperBound()(i) = m_Kp(i) * ((m_q_ubound(i) - m_buffer) - q(i)) - m_Kd(i) * v(i);
					  m_constraint.lowerBound()(i) = -m_velocity_limit_;

					  if (m_constraint.upperBound()(i) < -m_velocity_limit_)
						  m_constraint.upperBound()(i) = -m_velocity_limit_;

					  m_constraint.lowerBound()(i) = -m_velocity_limit_;
				  }
				  else {
					  m_constraint.upperBound()(i) = m_velocity_limit_;
					  m_constraint.lowerBound()(i) = -m_velocity_limit_;
				  }
			  }
			  MatrixXd A(m_robot.nv(), m_robot.nv());
			  A.setIdentity();
			  m_constraint.setMatrix(A);	
			  	
			  return m_constraint;
		  }
		  else if (m_robot.type() == 1) {
			  for (int i = 0; i < 2; i++) {
				  m_constraint.lowerBound()(i) = m_q_lbound(i);
				  m_constraint.upperBound()(i) = m_q_ubound(i);
			  }
			  for (int i = 2; i < m_robot.nv(); i++) {
				  if (q(i+3) < m_q_lbound(i) + m_buffer) {
					  m_constraint.lowerBound()(i) = m_Kp(i) * ((m_q_lbound(i) + m_buffer) - q(i+3)) - m_Kd(i) * v(i+3);
					  if (m_constraint.lowerBound()(i) > m_velocity_limit_)
						  m_constraint.lowerBound()(i) = m_velocity_limit_;

					  m_constraint.upperBound()(i) = m_velocity_limit_;
				  }
				  else if (q(i+3) > m_q_ubound(i) - m_buffer) {
					  //cout << q(i+3) <<"\t " << m_q_ubound(i) - m_buffer <<endl;
					  m_constraint.upperBound()(i) = m_Kp(i) * ((m_q_ubound(i) - m_buffer) - q(i+3)) - m_Kd(i) * v(i+3);
					  if (m_constraint.upperBound()(i) < -m_velocity_limit_)
						  m_constraint.upperBound()(i) = -m_velocity_limit_;

					  m_constraint.lowerBound()(i) = -m_velocity_limit_;
				  }
				  else {
					  m_constraint.upperBound()(i) = m_velocity_limit_;
					  m_constraint.lowerBound()(i) = -m_velocity_limit_;
				  }
			  }
			  //if (m_robot.getManipulability(0, m_robot.getJointPosition()) < 0.01) {
				 // VectorXd Jd = m_robot.getManipulabilityJacobian(0);
				 // for (int i = 0; i < dof / 2; i++) {
					//  m_constraint.lowerBound()(i + 2) = 0.000005 / Jd(i) * (m_robot.getManipulability(0, m_robot.getJointPosition()) - 0.005);
					//  if (m_constraint.lowerBound()(i + 2) > 2000.0)
					//	  m_constraint.upperBound()(i + 2) = m_constraint.lowerBound()(i + 2);
				 // }
			  //}				  

			  MatrixXd A(m_robot.nv(), m_robot.nv());
			  A.setIdentity();
			  m_constraint.setMatrix(A);

			  return m_constraint;
		  }
	  }
	  const ConstraintBase & TaskJointLimit::getConstraint() const{
		  return m_constraint;
	  }

	  void TaskJointLimit::setJointLimit(Cref_vectorXd q_low, Cref_vectorXd q_high) {
	      assert(q_low.size() == m_robot.nv() && q_high.size() == m_robot.nv());

		  for (int i = 0; i < q_low.size(); i++)
			  assert(q_low(i) <= q_high(i));

		  m_q_lbound = q_low;
		  m_q_ubound = q_high;
	  }

	  const  VectorXd & TaskJointLimit::mask() const {
		
	  }
	  void TaskJointLimit::mask(const VectorXd & mask) {
		 
	  }

	  const VectorXd & TaskJointLimit::Kp() { return m_Kp; }

	  const VectorXd & TaskJointLimit::Kd() { return m_Kd; }

	  void TaskJointLimit::Kp(Cref_vectorXd Kp)
	  {
		  assert(Kp.size() == m_robot.nv());
		  m_Kp = Kp;
	  }

	  void TaskJointLimit::Kd(Cref_vectorXd Kd)
	  {
		  assert(Kd.size() == m_robot.nv());
		  m_Kd = Kd;
	  }        
  }
}
