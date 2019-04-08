#include "vrep_bridge/vrep_bridge.h"
using namespace std;
using namespace Eigen;

void VRepBridge::simxErrorCheck(simxInt error)
{
	string errorMsg;
	switch (error)
	{
	case simx_error_noerror:
		return;	// no error
		break;
	case simx_error_timeout_flag:
		errorMsg = "The function timed out (probably the network is down or too slow)";
		break;
	case simx_error_illegal_opmode_flag:
		errorMsg = "The specified operation mode is not supported for the given function";
		break;
	case simx_error_remote_error_flag:
		errorMsg = "The function caused an error on the server side (e.g. an invalid handle was specified)";
		break;
	case simx_error_split_progress_flag:
		errorMsg = "The communication thread is still processing previous split command of the same type";
		break;
	case simx_error_local_error_flag:
		errorMsg = "The function caused an error on the client side";
		break;
	case simx_error_initialize_error_flag:
		errorMsg = "simxStart was not yet called";
		break;
	default:
		errorMsg = "Unknown error.";
		break;
	}

	cout << "[ERROR] An error is occured. code = " << error << endl;
	cout << " - Description" << endl;
	cout << " | " << errorMsg << endl;

	throw std::string(errorMsg);
}

void VRepBridge::dataInit()
{
	current_q_.resize(MOTORNUM);
	current_q_.setZero();
	current_qdot_.resize(MOTORNUM);
	current_qdot_.setZero();
	desired_q_.resize(MOTORNUM);
	desired_q_.setZero();
	desired_torque_.resize(MOTORNUM);
	desired_torque_.setZero();
	current_base_vel_.resize(4);
	desired_base_vel_.resize(4);

	_cntt = 0;

}

void VRepBridge::simInit()
{
	simxFinish(-1);
	clientID_ = simxStart("", -1000, true, true, 2000, 5);
	if (clientID_ < 0)
	{
		throw std::string("Failed connecting to remote API server. Exiting.");
	}


	simxErrorCheck(simxStartSimulation(clientID_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxSynchronous(clientID_, true));

	cout << "[INFO] V-Rep connection is established." << endl;

}

void VRepBridge::write()
{
	for (size_t i = 0; i < MOTORNUM; i++) {
		simxFloat velLimit;

		if (desired_torque_(i) >= 0.0)
			velLimit = 10e10f;
		else
			velLimit = -10e10f;

		simxSetJointTargetVelocity(clientID_, motorHandle_[i], velLimit, simx_opmode_streaming);
		simxSetJointForce(clientID_, motorHandle_[i], static_cast<float>(abs(desired_torque_(i))), simx_opmode_streaming);
	}

	for (size_t i = 0; i < 4; i++)
		simxSetJointTargetVelocity(clientID_, baseHandle_[i], desired_base_vel_(i), simx_opmode_streaming);

	cnt++;

}
void VRepBridge::read()
{
	// manipulator
	for (size_t i = 0; i < MOTORNUM; i++)
	{
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_[i], &data, simx_opmode_streaming);
		current_q_(i) = data;

		simxGetObjectFloatParameter(clientID_, motorHandle_[i], 2012, &data, simx_opmode_streaming);
		current_qdot_(i) = data;
	}


	// base vel 
	for (size_t i = 0; i < 4; i++)
	{
		simxFloat data;
		simxGetObjectFloatParameter(clientID_, baseHandle_[i], 2012, &data, simx_opmode_streaming);
		current_base_vel_(i) = data;
	}
	// base pos / ori
	simxFloat data[3];
	simxGetObjectPosition(clientID_, StateEstimator_, -1, data, simx_opmode_streaming);
	H_transform_.translation() << data[0], data[1], data[2];
	simxGetObjectOrientation(clientID_, StateEstimator_, -1, data, simx_opmode_streaming);
	euler_ << data[0], data[1], data[2];

	Matrix3d rot;
	rot = Eigen::AngleAxisd(data[2], Vector3d::UnitZ()) *Eigen::AngleAxisd(data[1], Vector3d::UnitY()) * Eigen::AngleAxisd(data[0], Vector3d::UnitX());
	H_transform_.linear() = rot;


	simxFloat data2[3];
	simxGetObjectVelocity(clientID_, StateEstimator_, data, data2, simx_opmode_streaming);
	H_vel_.linear() << data[0], data[1], data[2];
	H_vel_.angular() << data2[0], data2[1], data2[2];


	////////////////////////////////////
	simxFloat data3[3];
	simxGetObjectPosition(clientID_, maniCenter_, -1, data3, simx_opmode_streaming);
	Mani_transform_.translation() << data3[0], data3[1], data3[2];
	center_pos << data3[0], data3[1], data3[2];
	mobile_center << data3[0], data3[1], data3[2];

	simxGetObjectOrientation(clientID_, maniCenter_, -1, data, simx_opmode_streaming);
	euler2_ << data3[0], data3[1], data3[2];

	Matrix3d rot2;
	rot2 = Eigen::AngleAxisd(data3[2], Vector3d::UnitZ()) *Eigen::AngleAxisd(data3[1], Vector3d::UnitY()) * Eigen::AngleAxisd(data3[0], Vector3d::UnitX());
	Mani_transform_.linear() = rot2;

	simxFloat data4[3];
	simxGetObjectVelocity(clientID_, maniCenter_, data3, data4, simx_opmode_streaming);
	Mani_vel_.linear() << data3[0], data3[1], data3[2];
	Mani_vel_.angular() << data4[0], data4[1], data4[2];

}


void VRepBridge::getHandle()
{
	cout << "[INFO] Getting handles." << endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint1", &motorHandle_[0], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint2", &motorHandle_[1], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint3", &motorHandle_[2], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint4", &motorHandle_[3], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint5", &motorHandle_[4], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint6", &motorHandle_[5], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint7", &motorHandle_[6], simx_opmode_oneshot_wait)); // Left Arm

	simxErrorCheck(simxGetObjectHandle(clientID_, "joint_1", &baseHandle_[0], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "joint_2", &baseHandle_[1], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "joint_3", &baseHandle_[2], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "joint_4", &baseHandle_[3], simx_opmode_oneshot_wait));

	simxErrorCheck(simxGetObjectHandle(clientID_, "Dummy", &StateEstimator_, simx_opmode_oneshot_wait));

	simxErrorCheck(simxGetObjectHandle(clientID_, "mani_center", &maniCenter_, simx_opmode_oneshot_wait));

	cout << "[INFO] The handle has been imported." << endl;
}