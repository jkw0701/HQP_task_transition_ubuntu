#pragma once
#include <iostream>
#include <string>
#include <functional>
#include "utils/motion.h"
#include <Eigen/Dense>
using namespace std;

extern "C" {
#include "extApi.h"
}

const int MOTORNUM = dof;	/// < Depends on simulation envrionment

class VRepBridge
{
//private:
//	typedef std::function<void()> callfunc; // loop callback function

public:
	VRepBridge() : tick_(0)
	{
		dataInit();
		simInit();
		getHandle();
	}
	~VRepBridge()
	{
		simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
		simxFinish(clientID_);
	}

	bool simConnectionCheck()
	{
		return (simxGetConnectionId(clientID_) != -1);
	}
	void simLoop()
	{
		//loopCallbackFunc();
		tick_++;
		simxSynchronousTrigger(clientID_);
	}
	void write();
	void read();

	const simxInt getClientID() {
		return clientID_;
	}


public:
	int cnt = 0;
	simxFloat dist1_[3];

	simxFloat obs1_[3];
	simxFloat obs2_[3];

	simxFloat dist2_[3];
	simxFloat dist3_[3];	
	simxFloat dist4_[3];	
	simxFloat dist5_[3];	
	simxFloat dist6_[3];	
	Eigen::VectorXd current_q_;
	Eigen::VectorXd current_qdot_;
	Eigen::VectorXd desired_q_;
	Eigen::VectorXd desired_torque_;
	Eigen::VectorXd target_x_;

	Eigen::Vector3d force_;
	Eigen::Vector3d torque_;
	Eigen::Vector3d desired_obs_pos;
	Eigen::Vector3d target_pos;
	Eigen::Vector3d euler_;
	Eigen::Vector3d euler2_;
	Eigen::Vector3d center_pos;
	Eigen::Vector3d mobile_center;

	Eigen::VectorXd desired_base_vel_, current_base_vel_;

	Transform3d Mani_transform_;
	Transform3d H_transform_;
	HQP::MotionVector<double> H_vel_;
	HQP::MotionVector<double> Mani_vel_;


	const size_t getTick() { return tick_; }


	bool isSimulationRun = false;
	bool exitFlag = false;

	double Hz;
	int _cntt;

	simxInt dialog_handle;

private:
	simxInt clientID_;
	simxInt clientID_shared;
	simxInt motorHandle_[MOTORNUM];	/// < Depends on simulation envrionment
	simxInt baseHandle_[4];
	simxInt objectHandle_;
	simxInt StateEstimator_;
	simxInt Obstacle;
	simxInt Target;
	simxInt Collection_;
	simxInt maniCenter_;
	simxInt obstacle_;
	simxInt obstacle_2;


	simxInt distHandle[6];
	
	simxInt targetHandle_;

	size_t tick_;
	//callfunc loopCallbackFunc;

	void simxErrorCheck(simxInt error);
	void dataInit();
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
