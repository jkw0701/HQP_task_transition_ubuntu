#include "trajectories/trajectory-jointspace.h"
#include <iostream>
using namespace HQP::trajectories;

TrajectoryJointConstant::TrajectoryJointConstant(const std::string & name)
    :TrajectoryBase(name)
{}

TrajectoryJointConstant::TrajectoryJointConstant(const std::string & name, Cref_vectorXd ref)
    :TrajectoryBase(name)
{
    setReference(ref);
}

void TrajectoryJointConstant::setReference(Cref_vectorXd ref)
{
    m_sample.pos = ref;
    m_sample.vel.setZero(ref.size());
    m_sample.acc.setZero(ref.size());
}

unsigned int TrajectoryJointConstant::size() const
{
    return (unsigned int)m_sample.pos.size();
}

const TrajectorySample & TrajectoryJointConstant::operator()(double )
{
    return m_sample;
}

const TrajectorySample & TrajectoryJointConstant::computeNext()
{
    return m_sample;
}

void TrajectoryJointConstant::getLastSample(TrajectorySample & sample) const
{
    sample = m_sample;
}

bool TrajectoryJointConstant::has_trajectory_ended() const
{
    return true;
}
//// Trj cubic
TrajectoryJointCubic::TrajectoryJointCubic(const std::string & name)
	:TrajectoryBase(name)
{}

TrajectoryJointCubic::TrajectoryJointCubic(const std::string & name, Cref_vectorXd init_M, Cref_vectorXd goal_M, const double & duration, const double & stime)
	: TrajectoryBase(name)
{
	setGoalSample(goal_M);
	setInitSample(init_M);
	setDuration(duration);
	setStartTime(stime);

	//setReference(ref);
	//m_sample.resize(12, 6);
	//m_sample.pos.head<3>() = init_M.translation();
	//typedef Eigen::Matrix<double, 9, 1> Vector9;
	//m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
}
unsigned int TrajectoryJointCubic::size() const
{
	return (unsigned int)m_sample.pos.size();
}

const TrajectorySample & TrajectoryJointCubic::operator()(double)
{
	return m_sample;
}

const TrajectorySample & TrajectoryJointCubic::computeNext()
{
	if (m_time <= m_stime) {
		m_sample.pos = m_init;
		return m_sample;
	}
	else if (m_time >= m_stime + m_duration) {
		m_sample.pos = m_goal;

		return m_sample;
	}
	else {
		double a0, a1, a2, a3;
		VectorXd cubic_tra = m_init;
		for (int i = 0; i < m_init.size(); i++) {
			a0 = m_init(i);
			a1 = 0.0; //m_init.vel(i);
			a2 = 3.0 / pow(m_duration, 2) * (m_goal(i) - m_init(i));
			a3 = -1.0 * 2.0 / pow(m_duration, 3) * (m_goal(i) - m_init(i));

			cubic_tra(i) = a0 + a1 * (m_time - m_stime) + a2 * pow(m_time - m_stime, 2) + a3 * pow(m_time - m_stime, 3);
		}
		//cout << "cubic" << cubic_tra.transpose() <<endl;
		m_sample.pos = cubic_tra;
		return m_sample;
	}
}

void TrajectoryJointCubic::getLastSample(TrajectorySample & sample) const
{
	sample = m_sample;
}

bool TrajectoryJointCubic::has_trajectory_ended() const
{
	return true;
}

void TrajectoryJointCubic::setGoalSample(Cref_vectorXd goal_M)
{
	m_goal = goal_M;
}
void TrajectoryJointCubic::setInitSample(Cref_vectorXd init_M)
{
	m_init = init_M;
}
void TrajectoryJointCubic::setDuration(const double & duration)
{
	m_duration = duration;
}
void TrajectoryJointCubic::setCurrentTime(const double & time)
{
	m_time = time;
}
void TrajectoryJointCubic::setStartTime(const double & time)
{
	m_stime = time;
}

void TrajectoryJointCubic::setReference(const Cref_vectorXd ref) {
	m_sample.pos = ref;
	m_sample.vel.setZero(ref.size());
	m_sample.acc.setZero(ref.size());
}