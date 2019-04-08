#ifndef __HQP__TRAJECTOEIS__JOINT__
#define __HQP__TRAJECTOEIS__JOINT__

#include "trajectory-base.h"

#include <string>

namespace HQP {
	namespace trajectories {
		class TrajectoryJointConstant : public TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			TrajectoryJointConstant(const std::string & name);
			TrajectoryJointConstant(const std::string & name,Cref_vectorXd ref);

			unsigned int size() const;
			void setReference(Cref_vectorXd ref);
			void setgood() {};
			const TrajectorySample & operator()(double time);
			const TrajectorySample & computeNext();
			void getLastSample(TrajectorySample & sample) const;
			bool has_trajectory_ended() const;

		protected:
			VectorXd m_ref;
		};

		class TrajectoryJointCubic : public TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			TrajectoryJointCubic(const std::string & name);
			TrajectoryJointCubic(const std::string & name, Cref_vectorXd init_M, Cref_vectorXd goal_M, const double & duration, const double & stime);

			unsigned int size() const;
			const TrajectorySample & operator()(double time);
			const TrajectorySample & computeNext();
			void getLastSample(TrajectorySample & sample) const;
			bool has_trajectory_ended() const;
			void setReference(Cref_vectorXd ref);
			void setInitSample(Cref_vectorXd init_M);
			void setGoalSample(Cref_vectorXd goal_M);
			void setDuration(const double & duration);
			void setCurrentTime(const double & time);
			void setStartTime(const double & time);

		protected:
			VectorXd m_ref;
			VectorXd m_init, m_goal, m_cubic;
			double m_duration, m_stime, m_time;
		};
	}
}


#endif // __HQP__TRAJECTOEIS__JOINT__