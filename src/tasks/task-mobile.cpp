#include "tasks/task-mobile.h"
#include "utils/utils.h"

using namespace std;
namespace HQP
{
	namespace tasks
	{
		using namespace constraint;
		using namespace trajectories;

		TaskMobile::TaskMobile(const std::string & name, RobotModel & robot) : // only mobile orientation
			TaskMotion(name, robot),
			m_constraint(name, 6, robot.nv()),
			m_ref(12, 6)
		{
			m_v_ref.setZero();
			m_a_ref.setZero();
			m_M_ref.setIdentity();
			m_wMl.setIdentity();
			m_p_error_vec.setZero(6);
			m_v_error_vec.setZero(6);
			m_p.resize(12);
			m_v.resize(6);
			m_p_ref.resize(12);
			m_v_ref_vec.resize(6);
			m_Kp.setZero(6);
			m_Kd.setZero(6);
			m_a_des.setZero(6);
			m_J.setZero(6, robot.nv());
			m_ori_ctrl = false;
			m_beta = 1.0;
			transition_flag = false;
			m_old_sol.setZero(robot.nv());
		}

		int TaskMobile::dim() const
		{
			return 6;
		}

		const VectorXd & TaskMobile::Kp() const { return m_Kp; }

		const VectorXd & TaskMobile::Kd() const { return m_Kd; }

		void TaskMobile::Kp(Cref_vectorXd Kp)
		{
			assert(Kp.size() == 6);
			m_Kp = Kp;
		}

		void TaskMobile::Kd(Cref_vectorXd Kd)
		{
			assert(Kd.size() == 6);
			m_Kd = Kd;
		}

		void TaskMobile::setReference(TrajectorySample & ref)
		{
			m_ref = ref;
			m_M_ref.translation() = ref.pos.head<3>();
			typedef Eigen::Matrix<double, 3, 3> Matrix3;
			m_M_ref.linear() = Eigen::Map<const Matrix3>(&ref.pos(3), 3, 3);

			m_v_ref = MotionVector<double>(ref.vel);
			m_a_ref = MotionVector<double>(ref.acc);
		}
		void TaskMobile::setOnlyOriCTRL(bool onlyori) {
			m_ori_ctrl = onlyori;
			if (m_ori_ctrl)
				m_constraint.resize(3, m_robot.nv());
			else
				m_constraint.resize(6, m_robot.nv());

		}
		const TrajectorySample & TaskMobile::getReference() const
		{
			return m_ref;
		}

		const VectorXd & TaskMobile::position_error() const
		{
			return m_p_error_vec;
		}

		const VectorXd & TaskMobile::velocity_error() const
		{
			return m_v_error_vec;
		}

		const VectorXd & TaskMobile::position() const
		{
			return m_p;
		}

		const VectorXd & TaskMobile::velocity() const
		{
			return m_v;
		}

		const VectorXd & TaskMobile::position_ref() const
		{
			return m_p_ref;
		}

		const VectorXd & TaskMobile::velocity_ref() const
		{
			return m_v_ref_vec;
		}

		const VectorXd & TaskMobile::getDesiredAcceleration() const
		{
			return m_a_des;
		}

		VectorXd TaskMobile::getAcceleration(Cref_vectorXd dv) const
		{
			return m_constraint.matrix()*dv + m_drift.vector();
		}

		//Index TaskOperationalSpace::frame_id() const
		//{
		//  return m_frame_id;
		//}

		const ConstraintBase & TaskMobile::getConstraint() const
		{
			return m_constraint;
		}

		const ConstraintBase & TaskMobile::compute(const double t, Cref_vectorXd q, Cref_vectorXd v)
		{
			Transform3d oMi;
			MotionVector<double> v_frame;

			m_robot.getUpdateKinematics(q, v);
			oMi = m_robot.getMobileTransformation();
			v_frame = m_robot.getMobileVelocity();

			m_drift.setZero(); // check acc
							   //m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);

							   // Transformation from local to world
			m_wMl.linear() = oMi.linear();

			Transform3d b;
			b = m_M_ref.inverse() * oMi;
			m_p_error = log6(b);
			m_v_error = v_frame - m_v_ref;

			m_v_error = actinv(oMi, m_v_error.vector());

			m_p_error_vec = m_p_error.vector();
			m_v_error_vec = m_v_error.vector();

			m_p_ref.head<3>() = m_M_ref.translation();
			typedef Eigen::Matrix<double, 9, 1> Vector9;
			m_p_ref.tail<9>() = Eigen::Map<const Vector9>(&m_M_ref.rotation()(0), 9);

			m_v_ref_vec = m_v_ref.vector();

			m_p.head<3>() = oMi.translation();
			m_p.tail<9>() = Eigen::Map<const Vector9>(&oMi.rotation()(0), 9);

			m_v = v_frame.vector();

			m_a_des = -m_Kp.cwiseProduct(m_p_error.vector())
				- m_Kd.cwiseProduct(m_v_error.vector());
			//+ m_a_ref.actInv(m_wMl).vector();



			m_J = m_robot.getJacobian(7); //check world jacobian
			m_J.block(0, 2, 6, m_robot.nv() - 2).setZero();


			if (m_ori_ctrl) { // if you want to control mobile orientation only
				Vector3d b = m_a_des.tail(3);
				MatrixXd A = m_J.bottomRows(3);


				if (!transition_flag)
				{
					m_constraint.resize(3, m_robot.nv());
					m_constraint.setMatrix(A);
					m_constraint.setVector(m_beta*b);
				}
				else
				{
					m_constraint.resize(3, m_robot.nv());
					m_constraint.setMatrix(A);
					m_constraint.setVector((m_beta * b + (1.0 - m_beta) * A * m_old_sol));
				
				}
			}
			else {

				if (!transition_flag)
				{
					m_J.bottomRows(1).setZero();
					m_a_des(5) = 0.0;
					m_constraint.setMatrix(m_J);
					m_constraint.setVector(m_beta*m_a_des);
				}
				else
				{
					m_J.bottomRows(1).setZero();
					m_a_des(5) = 0.0;
					m_constraint.setMatrix(m_J);
					m_constraint.setVector(m_beta * m_a_des + (1.0 - m_beta) * m_J * m_old_sol);
				}
			}

			return m_constraint;



		}
	}
}