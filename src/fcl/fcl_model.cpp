#include "fcl/fcl_model.h"
#define TEST_RESOURCES_DIR "/home/kendrick/self-collision vrep model/obj_file/single_arm"
FILE *fp3 = fopen("SCA_data_distances_dual_arm.txt","w");


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

using namespace fcl;
using namespace HQP::fclmodel;
using namespace RigidBodyDynamics;
int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

std::vector<Contact> global_pairs;
std::vector<Contact> global_pairs_now;

FCL_MODEL::FCL_MODEL(HQP::robot::RobotModel & robot)
: m_robot(robot)
{
	distance_vec.resize(linkpair);
	distance_vec.setZero();

	for(int i=0;i<linkpair;i++)
	{
		Jaco[i].resize(6,dof);
		Jaco[i].setZero();
		Jaco_[i].resize(6, dof);
		Jaco_[i].setZero();
		Jaco_pos[i].resize(3, dof);
		Jaco_pos[i].setZero();
	}

	for (int i=0;i<linkpair;i++)
	verbose_[i] = true;
}

template<typename BV>
void getDistanceAndPosition(fcl::BVHModel<BV> m1,fcl::BVHModel<BV> m2, Transform3f& tf1, const Transform3f& tf2,
                   const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                   int qsize,
                   DistanceRes& distance_result,
                   bool verbose)
{

  Transform3f pose1(tf1), pose2(tf2);

  DistanceResult local_result;
  MeshDistanceTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2, DistanceRequest(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;
  distance(&node, NULL, qsize);
  distance_result.distance = local_result.min_distance;
  distance_result.p1 = local_result.nearest_points[0];
  distance_result.p2 = local_result.nearest_points[1];

}



void FCL_MODEL::initializeModel()
{

  boost::filesystem::path path(TEST_RESOURCES_DIR);
  
  loadOBJFile((path / "Link1.obj").string().c_str(), p[0], t[0]);
  loadOBJFile((path / "Link23.obj").string().c_str(), p[1], t[1]);
  loadOBJFile((path / "Link45.obj").string().c_str(), p[2], t[2]);
  loadOBJFile((path / "base_sca.obj").string().c_str(), p[3], t[3]);
  loadOBJFile((path / "obstacle.obj").string().c_str(), p[4], t[4]);
  loadOBJFile((path / "obstacle.obj").string().c_str(), p[5], t[5]);


  for (int i=0;i<modelnum;i++)
  {
  model[i].bv_splitter.reset(new BVSplitter<OBBRSS>(SPLIT_METHOD_MEAN));
  model[i].beginModel();
  model[i].addSubModel(p[i],t[i]);
  model[i].endModel();
  }
  
  Rot[3].setIdentity(); // for base link
  Trs[3].setValue(0.0, 0.0, 0.225);


}

void FCL_MODEL::checkDistancePairs()
{


  global_pairs.clear();
  global_pairs_now.clear();


  for (int i=0;i<modelnum;i++)
    tf[i].setTransform(Rot[i], Trs[i]);


  // Distance Tests between 6 pairs of links
  getDistanceAndPosition<OBBRSS>(model[0], model[2], tf[0], tf[2], p[0], t[0], p[4], t[4], SPLIT_METHOD_MEAN,20, Dis[0],verbose_[0]);
  getDistanceAndPosition<OBBRSS>(model[3], model[1], tf[3], tf[1], p[7], t[7], p[4], t[4], SPLIT_METHOD_MEAN,20, Dis[1],verbose_[4]);
  getDistanceAndPosition<OBBRSS>(model[3], model[2], tf[3], tf[2], p[7], t[7], p[5], t[5], SPLIT_METHOD_MEAN,20, Dis[2],verbose_[5]);
  getDistanceAndPosition<OBBRSS>(model[3], model[4], tf[3], tf[4], p[7], t[7], p[5], t[5], SPLIT_METHOD_MEAN,20, Dis[3],verbose_[5]);
  getDistanceAndPosition<OBBRSS>(model[3], model[5], tf[3], tf[5], p[7], t[7], p[5], t[5], SPLIT_METHOD_MEAN,20, Dis[4],verbose_[5]);

}
void FCL_MODEL::updateLinkInfo(VectorXd q){
	q_ = q;

	for (int i=0;i<dof;i++)
	{
	 joint_pos[dof-1-i] = CalcBodyToBaseCoordinates(*m_robot.getModel(true), q_,  m_robot.getsubBodyID(dof-1-i), Vector3d::Zero(), true); 
	 link_rot[dof-1-i] = CalcBodyWorldOrientation(*m_robot.getModel(false), q_,  m_robot.getBodyID(dof-1-i), true).transpose();
  	}
	
	link_rot_fcl[0] = link_rot[0];
	link_rot_fcl[1] = link_rot[1];
	link_rot_fcl[2] = link_rot[3];
	link_rot_fcl[3] = base_rot;
	link_rot_fcl[4].setIdentity();
	link_rot_fcl[5].setIdentity();

	link_com_fcl[0] = CalcBodyToBaseCoordinates(*m_robot.getModel(false), q_, m_robot.getsubBodyID(0), Vector3d(0.0, 0.0, -0.1715), true);
	link_com_fcl[1] = CalcBodyToBaseCoordinates(*m_robot.getModel(false), q_, m_robot.getsubBodyID(1), Vector3d(0.0, 0.0, 0.1535), true);
	link_com_fcl[2] = CalcBodyToBaseCoordinates(*m_robot.getModel(false), q_, m_robot.getsubBodyID(3), Vector3d(-0.0309, -0.0064, 0.3364), true);
	base_pos(2) -= 0.0303;
	//cout << link_com_fcl[2].transpose()<<endl;
	link_com_fcl[3] = base_pos;
	link_com_fcl[4] = obs_pos;
	link_com_fcl[5] = obs_pos2;


	for (int i = 0; i < modelnum; i++)
	{
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
			{
				Rot[i](j, k) = link_rot_fcl[i](j, k);
				temp[k] = link_com_fcl[i](k);
			}
		Trs[i].setValue(temp[0], temp[1], temp[2]);
	}


}

void FCL_MODEL::getClosestJacobian(){

	for (int i = 0; i < linkpair; i++)
	{
		distance_vec(i) = Dis[i].distance;
		for (int j = 0; j < 3; j++)
		{
			P_b[i](j) = Dis[i].p1[j];
			P_a[i](j) = Dis[i].p2[j];
		}
	}
	//cout << Dis[4].distance << endl;



	double k_obs = 1.0;
	for(int i=0;i<3;i++){
	// k_obs의 영향은 없음. 왜냐하면 밑에서 정규화를 시켜주기 때문임.
	// 0.3 > pose 1
	m_f_obs[i] = k_obs*h_factor_dis(distance_vec(i), 0.40, 0.0)*(P_a[i] - P_b[i]) / (P_a[i] - P_b[i]).norm();
	m_f_obs_val[i] = m_f_obs[i].norm();
	h_value[i] = h_factor_dis(distance_vec(i), 0.15, 0.15 - 0.15);
	}
	

	// m_left_for[0] = m_f_obs[0] + m_f_obs[2];
	// m_left_pt[0] = ( m_f_obs_val[0]*P_a[0] + m_f_obs_val[2]*P_a[2] ) / (m_f_obs_val[0] + m_f_obs_val[2]);
	m_left_for[0] = m_f_obs[2];
	m_left_pt[0] =  P_a[2] ;
	m_obs_h_value[0] = h_value[2] ;

	m_left_for[1] = m_f_obs[1] ;
	m_left_pt[1] = P_a[1];
	m_obs_h_value[1] = h_value[1] ;

	double projection = (m_left_pt[0] - joint_pos[3]).dot(joint_pos[4] - joint_pos[3]) / (joint_pos[4] - joint_pos[3]).norm() / (joint_pos[4] - joint_pos[3]).norm();
	if (projection > 0)
	{
		com_pos_dis[0] = m_robot.getJointFramePosition(4);
	}
	else if (projection < 0)
	{
		com_pos_dis[0] = Vector3d::Zero();
	}
	else
	{
		com_pos_dis[0] = m_robot.getJointFramePosition(4) * (m_left_pt[0] - joint_pos[3]).dot(joint_pos[4] - joint_pos[3]) / (joint_pos[4] - joint_pos[3]).norm() / (joint_pos[4] - joint_pos[3]).norm();
	}
	//cout << projection << " \t"<<com_pos_dis[0].transpose() <<endl;

	//com_pos_dis[0] = m_robot.getJointFramePosition(4)*(m_left_pt[0]-joint_pos[3]).dot(joint_pos[4]-joint_pos[3])/(joint_pos[4] - joint_pos[3]).norm()/(joint_pos[4] - joint_pos[3]).norm()  ;
	com_pos_dis[1] = m_robot.getJointFramePosition(2)*(m_left_pt[1]-joint_pos[1]).dot(joint_pos[2]-joint_pos[1])/(joint_pos[2] - joint_pos[1]).norm()/(joint_pos[2] - joint_pos[1]).norm() ;



	CalcPointJacobian6D(*m_robot.getModel(false), q_, m_robot.getBodyID(3), com_pos_dis[0], Jaco[0], true);
	CalcPointJacobian6D(*m_robot.getModel(false), q_, m_robot.getBodyID(1), com_pos_dis[1], Jaco[1], true);

	for (int i = 0; i<2; i++){
		for(int j=0;j<linkpair-1;j++)
		Jaco_[j].block<3, dof>(i * 3, 0) = Jaco[j].block<3, dof>(3 - i * 3, 0);
	}

	for(int i=0;i<linkpair-1;i++)
	Jaco_pos[i] = Jaco_[i].block<3,dof>(0,0);

	
	m_J_obs.resize(3,9);
	m_J_obs.setZero();
	//cout << Jaco_pos[0] <<endl;
	m_J_obs.topRightCorner(3,7) = Jaco_pos[0];
	m_u_obs = (m_left_for[0]) / (m_left_for[0]).norm();
	m_u_obs_vel = m_J_obs * m_robot.getRealJointVelocity();
	m_obs_activation = m_obs_h_value[0];
	m_a_obs = m_obs_activation * (25.0 * m_left_for[0]  - 15.0 * m_u_obs_vel);

	m_J_obs2 = Jaco_pos[1];
	m_u_obs2 = (m_left_for[1]) / (m_left_for[1]).norm();
	m_u_obs_vel2 = m_J_obs2 * m_robot.getRealJointVelocity();
	m_obs_activation2 = m_obs_h_value[1];
	m_a_obs2 = m_obs_activation2 * (25.0 * m_left_for[1]  - 15.0 * m_u_obs_vel2);

	fprintf(fp3, "%f\t %f\t %f\t %f\t %f\t \n", distance_vec(0), distance_vec(1), distance_vec(2), distance_vec(3),distance_vec(4) );


}
