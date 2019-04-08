#ifndef __ROBOT_MODEL__
#define __ROBOT_MODEL__

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_config.h>
//#include <rbdl/rbdl.h>
#include "utils/motion.h"
#include "fwd.h"

#include <iostream>
#include <memory>
#include <fstream>

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;

namespace HQP {
	namespace robot {

		 enum class Type : unsigned int {
		 	Manipulator = 0,
		 	MobileManipulator = 1,
		 	Humanoid = 2
		 };

		class RobotModel
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				RobotModel(int robottype);
			~RobotModel();

			void getUpdateKinematics(const VectorXd & q, const VectorXd & qdot);
			void getClosestPoint(Vector3d Obs_pos);

			const MatrixXd & getJacobian(const int & frame_id) {
				Jacobian(frame_id);
				return m_J_;
			}
			const Vector3d & getPosition(const int & frame_id) {
				Position(frame_id);
				return m_pos_;
			}
			const MatrixXd & getOrientation(const int & frame_id) {
				Orientation(frame_id);
				return m_Ori_;
			}
			const Transform3d & getTransformation(const int & frame_id) {
				Transformation(frame_id);
				return m_Trans_;
			}
			const VectorXd & getNLEtorque() {
				NLEtorque();
				return m_NLE_;
			}
			const MatrixXd & getMassMatrix() {
				MassMatrix();
				return m_Mass_mat_;
			}
			const unsigned int & na() {
				return m_na_;
			}
			const unsigned int & nv() {
				return m_nv_;
			}
			const unsigned int & nq() {
				return m_nq_;
			}
			const int & type() {
				return m_robot_type_;
			}
			const MotionVector<double> & getPointVeloecity(const int & frame_id) {
				PointVelocity(frame_id);
				return m_p_dot_;
			}
			const VectorXd & getRealJointPosition() {
				q_real_.tail(m_nv_) = q_rbdl_.tail(m_nv_);
				return q_real_;
			}
			const VectorXd & getRealJointVelocity() {
				qdot_real_.tail(m_nv_) = qdot_rbdl_.tail(m_nv_);
				return qdot_real_;
			}
			const Transform3d & getMobileTransformation() {
				return m_base_;
			}
			const MotionVector<double> & getMobileVelocity() {
				return m_mobile_dot_;
			}
			const double & getManipulability(const int & i, const VectorXd & q) {
				Manipulability(q);
				return m_manipulability_[i];
			}
			const VectorXd & getManipulabilityJacobian(const int & i) {
				ManipulabilityJacobian();
				return m_J_manipulability_[i];
			}
			const Math::VectorNd & getJointPosition() {
				return q_rbdl_;
			}
			const double & getShortestDistance() {
				return shortest_dis;
			}
			const Vector3d & getAvoidVector() {
				return Obs_avoid;
			}
			const MatrixXd & getAvoidJaco() {
				return Jaco_pos;
			}
			const MatrixXd & getSelectionMatrix() {
				return m_selection_;
			}
			const Vector3d & getPointVel() {
				return Point_vel;
			}
			const Vector3d & getClosestPoint() {
				return Closest_point;
			}
			const Vector3d & getUnitVector() {
				return Unit_Task;
			}
			void getMobilePos(Vector3d & mob_pos) {
				m_mob_pos = mob_pos;
			}
			///////////////////////// FCL /////////////////////////////

			const Vector3d & getClosestCoMPosition(const int & i) {
				return com_pos_dis[i];
			}
			// const Vector3d & getCoMPosition(const int & i){
			// 	return com_pos_[i];
			// }			
			const Vector3d & getJointFramePosition(const int &i) {
				return joint_position_local_[i];
			}
			const unsigned int & getBodyID(const int &i) {
				return body_id_sim[i];
			}
			const unsigned int & getsubBodyID(const int &i) {
				return body_id_sub[i];
			}
			const shared_ptr<Model> & getModel(bool type) {
				if (type)
					return model_sub;
				else
					return model_sim;
			}




		private:
			void Jacobian(const int & frame_id);
			void Position(const int & frame_id);
			void Orientation(const int & frame_id);
			void Transformation(const int & frame_id);
			void NLEtorque();
			void MassMatrix();
			void PointVelocity(const int & frame_id);
			void setRobot();
			void Manipulability(const VectorXd &  q);
			void ManipulabilityJacobian();
			/////////////////////////////////////////////////////////////////
			unsigned int body_id_sub[dof];
			unsigned int body_id_sim[dof];
			Math::Vector3d com_pos_sub[dof];
			Math::Vector3d com_pos_dis[6];
			shared_ptr<Model> model_sub;
			shared_ptr<Model> model_sim;
			Body body_sub[dof];
			Body body_sim[dof];

			shared_ptr<Model> model_;
			Body body_[dof];
			Body virtual_body_[6];
			Body base_;

			Joint joint_[dof];
			Joint virtual_joint_[6];

			VectorXd q_;
			VectorXd qdot_;

			VectorXd q_real_;
			VectorXd qdot_real_;

			double mass_[dof];
			Math::Vector3d axis_[dof];
			Math::Vector3d inertia_[dof];
			Math::Vector3d joint_position_global_[dof];
			Math::Vector3d joint_position_local_[dof];
			Math::Vector3d com_position_[dof];
			Math::SpatialVector p_dot_;

			// Obstacle avoidance
			Math::Vector3d joint_pos[dof + 1];
			Math::Vector3d joint_pos_diff[dof];
			Math::Vector3d joint_to_obs[dof];
			double dot_product[dof];
			double dot_product_joint[dof];
			Math::Vector3d closest_on_link[dof];
			VectorXd distance_vector;
			MatrixXd Closest_Jaco;
			MatrixXd Jaco_pos;
			MatrixXd Jaco_temp;
			Vector3d Obs_avoid;
			double shortest_dis;
			Vector3d Point_vel;
			Vector3d Closest_point;
			Vector3d Unit_Task;

			Math::VectorNd q_rbdl_;
			Math::VectorNd qdot_rbdl_;
			Math::VectorNd qddot_rbdl_;
			Math::VectorNd tau_;

			unsigned int body_id_[dof];
			unsigned int base_id_; // virtual joint
			unsigned int virtual_body_id_[6];

			MatrixXd m_Mass_mat_;
			VectorXd m_NLE_;
			Vector3d m_pos_;
			Matrix3d m_Ori_;
			MatrixXd m_J_;
			MotionVector<double> m_p_dot_;
			MotionVector<double> m_mobile_dot_;
			MatrixXd m_selection_;
			MatrixXd m_selection_dot_, m_Mass_virtual_mat_;

			Transform3d m_Trans_;
			Transform3d m_base_;

			Vector3d m_mob_pos;
			int m_robot_type_;

			double m_manipulability_[2];
			VectorXd m_J_manipulability_[2];
			MatrixXd m_J_EE[2];

		protected:
			unsigned int m_nv_;
			unsigned int m_na_;
			unsigned int m_nq_;

		};
	}
}

#endif