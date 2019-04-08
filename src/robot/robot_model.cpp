#include "robot/robot_model.h"
#include <vector>
#include "utils/utils.h"

using namespace std;

HQP::robot::RobotModel::RobotModel(int robottype) {
	// model_ = new Model();
	// model_->gravity = Eigen::Vector3d(0., 0., -9.81);

	m_robot_type_ = robottype;
	if (m_robot_type_ == 0) {
		m_nq_ = dof;
		m_nv_ = dof;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 1) {
		m_nq_ = dof + 2;
		m_nv_ = dof + 2;
		m_na_ = dof;
	}
	else if (m_robot_type_ == 2) {
		m_nq_ = dof + 6;
		m_nv_ = dof + 6;
		m_na_ = dof;
	}

	q_rbdl_.resize(m_na_ + 5);
	qdot_rbdl_.resize(m_na_ + 5);
	q_rbdl_.setZero();
	qdot_rbdl_.setZero();

	qddot_rbdl_.resize(m_na_ + 5);
	qddot_rbdl_.setZero();

	m_NLE_.resize(m_na_ + 2); // 
	m_NLE_.setZero();
	m_Mass_mat_.resize(m_na_ + 2, m_na_ + 2); // 
	m_Mass_mat_.setZero();
	m_J_.resize(6, m_na_ + 2); // 
	m_J_.setZero();

	m_Ori_.resize(3, 3);
	m_pos_.setZero();
	m_Ori_.setZero();
	m_Trans_.linear().setZero();
	m_Trans_.translation().setZero();

	tau_.resize(m_na_ + 5);
	tau_.setZero();

	q_real_.resize(m_nv_);
	q_real_.setZero();
	qdot_real_.resize(m_nv_);
	qdot_real_.setZero();

	m_selection_.resize(5 + m_na_, m_na_ + 2);
	m_selection_.setZero();
	m_selection_dot_.resize(5 + m_na_, m_na_ + 2);
	m_selection_dot_.setZero();
	m_Mass_virtual_mat_.resize(5 + m_na_, 5 + m_na_);
	m_Mass_virtual_mat_.setZero();

	distance_vector.resize(dof);
	Closest_Jaco.resize(6, dof);
	Jaco_temp.resize(6, dof);
	Jaco_pos.resize(3, dof);

	setRobot();
}
HQP::robot::RobotModel::~RobotModel() {

}

void HQP::robot::RobotModel::setRobot() {
	model_ = make_shared<Model>();
	model_sub = make_shared<Model>();
	model_sim = make_shared<Model>();

	model_->gravity = Vector3d(0., 0, -9.81);
	model_sub->gravity = Vector3d(0., 0., -9.81);
	model_sim->gravity = Vector3d(0., 0., -9.81);

	// for floating base //
	virtual_body_[0] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	virtual_joint_[0] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitX());
	virtual_body_[1] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	virtual_joint_[1] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitY());
	virtual_body_[2] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	virtual_joint_[2] = Joint(Math::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

	virtual_body_id_[0] = model_->AddBody(0, Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[0], virtual_body_[0]);
	virtual_body_id_[1] = model_->AddBody(virtual_body_id_[0], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[1], virtual_body_[1]);

	virtual_body_id_[0] = model_sub->AddBody(0, Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[0], virtual_body_[0]);
	virtual_body_id_[1] = model_sub->AddBody(virtual_body_id_[0], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[1], virtual_body_[1]);

	virtual_body_id_[0] = model_sim->AddBody(0, Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[0], virtual_body_[0]);
	virtual_body_id_[1] = model_sim->AddBody(virtual_body_id_[0], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[1], virtual_body_[1]);

	double mass = 30.0;
	base_ = Body(mass, Math::Vector3d(0.0, 0.0, 0.0), Math::Vector3d(0.5, 100.0, 100.0));
	base_id_ = model_->AddBody(virtual_body_id_[1], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[2], base_); //body frame = joint frame
	base_id_ = model_sub->AddBody(virtual_body_id_[1], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[2], base_); //body frame = joint frame
	base_id_ = model_sim->AddBody(virtual_body_id_[1], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[2], base_); //body frame = joint frame


																															   //// for wheel
	virtual_body_[3] = Body(2.6 *2.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.001, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1));
	virtual_joint_[3] = Joint(JointTypeRevolute, -Eigen::Vector3d::UnitY());
	virtual_body_[4] = Body(2.6*2.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.001, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1));
	virtual_joint_[4] = Joint(JointTypeRevolute, 1.0*Eigen::Vector3d::UnitY());

	model_->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, 0.25, 0.165)), virtual_joint_[3], virtual_body_[3]);
	model_->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, -0.25, 0.165)), virtual_joint_[4], virtual_body_[4]);

	model_sub->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, 0.25, 0.165)), virtual_joint_[3], virtual_body_[3]);
	model_sub->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, -0.25, 0.165)), virtual_joint_[4], virtual_body_[4]);

	model_sim->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, 0.25, 0.165)), virtual_joint_[3], virtual_body_[3]);
	model_sim->AddBody(base_id_, Math::Xtrans(Math::Vector3d(0.0, -0.25, 0.165)), virtual_joint_[4], virtual_body_[4]);

	////////////////////////////////////////////////////////////////////////////////////////////////////


	for (int i = 0; i < dof; i++) {
		mass_[i] = 1.0;
		inertia_[i] = Vector3d(1.0, 1.0, 1.0);
	}

	axis_[0] = Eigen::Vector3d::UnitZ();
	axis_[1] = Eigen::Vector3d::UnitY();
	axis_[2] = Eigen::Vector3d::UnitZ();
	axis_[3] = -1.0*Eigen::Vector3d::UnitY();
	axis_[4] = Eigen::Vector3d::UnitZ();
	axis_[5] = -1.0*Eigen::Vector3d::UnitY();
	axis_[6] = -1.0*Eigen::Vector3d::UnitZ();


	joint_position_global_[0] = Eigen::Vector3d(0.3502, 0.0, 0.7081) ;
	joint_position_global_[1] = Eigen::Vector3d(0.3502, 0.0, 0.7080) ;
	joint_position_global_[2] = Eigen::Vector3d(0.3503, 0.0, 1.0240) ;
	joint_position_global_[3] = Eigen::Vector3d(0.4328, 0.0, 1.0240) ;
	joint_position_global_[4] = Eigen::Vector3d(0.3504, 0.0, 1.4080) ;
	joint_position_global_[5] = Eigen::Vector3d(0.3504, 0.0, 1.4080) ;
	joint_position_global_[6] = Eigen::Vector3d(0.4384, 0.0, 1.4080) ;


	joint_position_local_[0] = joint_position_global_[0];

	for (int i = 1; i < dof; i++)
		joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i - 1];

	com_position_[0] = Vector3d(0.3502, -0.0345, 0.6325) ;
	com_position_[1] = Vector3d(0.3504, 0.0345, 0.7844) ;
	com_position_[2] = Vector3d(0.3837, 0.0267, 0.9826) ;
	com_position_[3] = Vector3d(0.3834, -0.0265, 1.0665) ;
	com_position_[4] = Vector3d(0.3517, 0.0424, 1.2993) ;
	com_position_[5] = Vector3d(0.3925, -0.0102, 1.4232) ;
	com_position_[6] = Vector3d(0.4504, -0.0119, 1.3286) ;

	for (int i = 0; i < dof; i++)
		com_position_[i] -= joint_position_global_[i];


	for (int i = 0; i < dof; i++) {
		body_sim[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		body_sub[i] = Body(mass_[i], Vector3d::Zero(), inertia_[i]);


		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 0) {
			body_id_sim[i] = model_sim->AddBody(base_id_, Math::Xtrans(joint_position_local_[i]), joint_[i], body_sim[i]);
			body_id_sub[i] = model_sub->AddBody(base_id_, Math::Xtrans(joint_position_local_[i]), joint_[i], body_sub[i]);
		}
		else {
			body_id_sim[i] = model_sim->AddBody(body_id_sim[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_sim[i]);
			body_id_sub[i] = model_sub->AddBody(body_id_sub[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_sub[i]);
		}
	}



	for (int i = 0; i < dof; i++) {
		body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 0)
			body_id_[i] = model_->AddBody(base_id_, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	}

}
void HQP::robot::RobotModel::Jacobian(const int & frame_id) { //?
	MatrixXd J_temp(6, m_na_ + 5);
	J_temp.setZero();
	CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], J_temp, true);

	m_J_ = J_temp * m_selection_;

	J_temp = m_J_;
	for (int i = 0; i < 2; i++) {
		m_J_.block(i * 3, 0, 3, m_na_ + 2) = J_temp.block(3 - i * 3, 0, 3, m_na_ + 2);
	}


}
void HQP::robot::RobotModel::Position(const int & frame_id) { // for mobile
	m_pos_ = CalcBodyToBaseCoordinates(*model_, q_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1], true);
}
void HQP::robot::RobotModel::Orientation(const int & frame_id) { // for mobile
	m_Ori_ = CalcBodyWorldOrientation(*model_, q_rbdl_, body_id_[frame_id - 1], true).transpose();
}
void HQP::robot::RobotModel::Transformation(const int & frame_id) { // for mobile
	Position(frame_id);
	Orientation(frame_id);
	m_Trans_.linear() = m_Ori_;
	m_Trans_.translation() = m_pos_;
}
void HQP::robot::RobotModel::NLEtorque() { // for mobile
	VectorXd NLE_virtual(5 + m_na_);
	MassMatrix();
	InverseDynamics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_, tau_);

	for (int i = 0; i < 5 + m_na_; i++)
		NLE_virtual(i) = tau_(i);

	VectorXd q_real_dot(9);
	q_real_dot.setZero();
	q_real_dot.head(2) = qdot_rbdl_.head(2);

	m_NLE_ = m_selection_.transpose() * NLE_virtual + m_selection_.transpose() * m_Mass_virtual_mat_ * m_selection_dot_ * q_real_dot;
}
void HQP::robot::RobotModel::MassMatrix() { // for mobile
	MatrixXd Mass_virtual(5 + m_na_, 5 + m_na_);
	Mass_virtual.setZero();
	CompositeRigidBodyAlgorithm(*model_, q_rbdl_, m_Mass_virtual_mat_, true);
	m_Mass_mat_ = m_selection_.transpose() * m_Mass_virtual_mat_ * m_selection_;

}
void HQP::robot::RobotModel::Manipulability(const VectorXd &  q) {
	Jacobian(7);
	m_manipulability_[0] = sqrt((m_J_* m_J_.transpose()).determinant());
}
void HQP::robot::RobotModel::ManipulabilityJacobian() {
	m_J_manipulability_[0].resize(dof / 2);
	VectorXd q = q_rbdl_;
	Manipulability(q);
	const double mani_0 = m_manipulability_[0];
	const double h = 0.0000001;
	for (int i = 0; i < dof; i++) {
		q(i) += h;
		Manipulability(q);
		m_J_manipulability_[0](i) = (m_manipulability_[0] - mani_0) / h;
		q = q_rbdl_;
	}
}
void HQP::robot::RobotModel::getUpdateKinematics(const VectorXd & q, const VectorXd & qdot) { // for mobile
	q_rbdl_ = q;
	if (q(2) < 0.0)
		q_rbdl_(2) = M_PI * 2 + q(2);

	qdot_rbdl_ = qdot;
	qddot_rbdl_.setZero();
	UpdateKinematics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_);

	// for mobile
	m_base_.setIdentity();
	m_base_.rotate(AngleAxisd(q(2), Vector3d::UnitZ()));
	//m_base_.translation().head(2) = q_rbdl_.head(2);
	m_base_.translation()(0) = m_mob_pos(0);
	m_base_.translation()(1) = m_mob_pos(1);
	
	m_mobile_dot_.setZero();
	m_mobile_dot_.linear().head(2) = qdot.head(2);
	m_mobile_dot_.angular()(2) = qdot(2);

	// for selection matrix
	double r = 0.165, b = 0.5, d = 0.3, c = r / (2.0 * b);

	m_selection_.bottomRightCorner(m_na_ + 2, m_na_ + 2).setIdentity();
	m_selection_(0, 0) = c * (b*cos(q_rbdl_(2)) + d * sin(q_rbdl_(2)));
	m_selection_(0, 1) = c * (b*cos(q_rbdl_(2)) - d * sin(q_rbdl_(2)));
	m_selection_(1, 0) = c * (b*sin(q_rbdl_(2)) - d * cos(q_rbdl_(2)));
	m_selection_(1, 1) = c * (b*sin(q_rbdl_(2)) + d * cos(q_rbdl_(2)));

	m_selection_(2, 0) = -c;
	m_selection_(2, 1) = c;
	m_selection_(4, 0) = 1.0;
	m_selection_(3, 1) = 1.0;

	m_selection_dot_(0, 0) = c * (-b * sin(q_rbdl_(2)) + d * cos(q_rbdl_(2)));
	m_selection_dot_(0, 1) = c * (-b * sin(q_rbdl_(2)) - d * cos(q_rbdl_(2)));
	m_selection_dot_(1, 0) = c * (b*cos(q_rbdl_(2)) + d * sin(q_rbdl_(2)));
	m_selection_dot_(1, 1) = c * (b*cos(q_rbdl_(2)) - d * sin(q_rbdl_(2)));

	m_selection_dot_ *= qdot(2);

}

void HQP::robot::RobotModel::PointVelocity(const int & frame_id) {
	p_dot_ = CalcPointVelocity6D(*model_, q_rbdl_, qdot_rbdl_, body_id_[frame_id - 1], com_position_[frame_id - 1]);
	m_p_dot_.angular() = p_dot_.head<3>();
	m_p_dot_.linear() = p_dot_.tail<3>();
}

void HQP::robot::RobotModel::getClosestPoint(Vector3d Obs_pos) {

	// Joint Position
	for (int i = 0;i<7;i++) {
		joint_pos[6 - i] = CalcBodyToBaseCoordinates(*model_sub, q_rbdl_, body_id_sub[6 - i], Vector3d::Zero(), true);
		//	 cout << 6-i <<"\t"<<joint_pos[6-i].transpose() <<endl; 
	}
	joint_pos[7] = CalcBodyToBaseCoordinates(*model_sub, q_rbdl_, body_id_sub[6], com_position_[6], true);

	// From n-1 to n joint position
	for (int i = 0;i<7;i++) {
		joint_pos_diff[i] = joint_pos[i + 1] - joint_pos[i];
		//	cout << i <<"\t"<<joint_pos_diff[i].transpose() << endl;
	}


	// Closest Point on n-th link from Obstacle

	for (int i = 0;i<dof;i++) {
		joint_to_obs[i] = Obs_pos - joint_pos[i];
		dot_product[i] = joint_to_obs[i].dot(joint_pos_diff[i]);
		dot_product_joint[i] = joint_pos_diff[i].dot(joint_pos_diff[i]);

		if (dot_product[i] / dot_product_joint[i]>1.0) {
			closest_on_link[i] = joint_pos[i + 1];
		}
		else if (dot_product[i] / dot_product_joint[i]<0) {
			closest_on_link[i] = joint_pos[i];
		}
		else {
			closest_on_link[i] = joint_pos[i] + dot_product[i] / dot_product_joint[i] * joint_pos_diff[i];
		}
		distance_vector(i) = (closest_on_link[i] - Obs_pos).norm();
	}
	distance_vector(0) = 100.0;
	distance_vector(4) = 100.0;
	//cout << distance_vector.transpose() <<endl;	

	int min_coeff;
	distance_vector.minCoeff(&min_coeff);
	//cout << min_coeff <<endl;

	double dist = distance_vector(min_coeff);

	Vector3d temp;
	temp = joint_position_local_[min_coeff + 1] * (closest_on_link[min_coeff] - joint_pos[min_coeff]).dot(joint_pos[min_coeff + 1] - joint_pos[min_coeff]) / (joint_pos[min_coeff + 1] - joint_pos[min_coeff]).norm() / (joint_pos[min_coeff + 1] - joint_pos[min_coeff]).norm();
	CalcPointJacobian6D(*model_, q_rbdl_, body_id_[min_coeff], temp, Jaco_temp, true);

	for (int i = 0; i<2; i++) {
		Closest_Jaco.block<3, dof>(i * 3, 0) = Jaco_temp.block<3, dof>(3 - i * 3, 0);
	}
	Jaco_pos = Closest_Jaco.block < 3, dof>(0, 0);
	shortest_dis = distance_vector(min_coeff);
	Unit_Task = (closest_on_link[min_coeff] - Obs_pos) / (closest_on_link[min_coeff] - Obs_pos).norm();

	Closest_point = closest_on_link[min_coeff];
	Point_vel = Jaco_pos * qdot_rbdl_;

	Obs_avoid = h_factor_dis(shortest_dis, 0.18, 0.05)*(40 * Unit_Task - 10.0*Point_vel);




}