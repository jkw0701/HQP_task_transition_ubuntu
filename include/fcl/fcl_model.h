#ifndef FCL_MODEL_H
#define FCL_MODEL_H

#include "fcl_utility.h"
#include "robot/robot_model.h"
#include "utils/utils.h"


#define linkpair 5
#define modelnum 6

using namespace fcl;
using namespace std;

namespace HQP{
    namespace fclmodel{
        class FCL_MODEL
        {
        public:
            FCL_MODEL(HQP::robot::RobotModel & robot);
            ~FCL_MODEL(){};
            void initializeModel();
            void checkDistancePairs();
            void updateLinkInfo(VectorXd q);
            void getClosestJacobian();

            const VectorXd & getDistanceVector(){
                return distance_vec;
            }
            const MatrixXd & getTaskJaco(){
                return Jaco_avoid;
            }
            const Vector3d & getTaskVector(){
                return x_avoid;
            }
            const Vector3d & getClosestPointA(const int i){
                return P_a[0];
            }
            const Vector3d & getClosestPointB(const int i){
                return P_a[1];
            }
          const Vector3d & getClosestPointC(const int i){
                return P_a[2];
            }
            const Vector3d & getClosestPointD(const int i){
                return P_b[2];
            }
            const Vector3d & getClosestPointE(const int i){
                return P_b[3];
            }
          const Vector3d & getClosestPointF(const int i){
                return P_b[2];
            } 
          const Vector3d & getClosestPointG(const int i){
                return P_b[3];
            }         
          const Vector3d & getClosestPointH(const int i){
                return P_b[4];
            }                                                 
            const double & getClosestDistance(){
                return distance_vec(min_index);
            }
			const double & getDistanceMobandMani(){
				return Dis[2].distance;
			}
			const double & getDistanceMobandObs(){
				return Dis[3].distance;
			}
			const double & getDistanceMobandObs2(){
				return Dis[4].distance;
			}

			const MatrixXd & getObsJacobian() { return m_J_obs; }
			const Vector3d & getObsUnitVector() { return m_u_obs; }
			const Vector3d & getObsErrorVector() { return m_a_obs; }
			const double & getObsMinimumDist() { return m_dist; }
			const double & getObsActivation() { return m_obs_activation; }

			const MatrixXd & getObsJacobian2() { return m_J_obs2; }
			const Vector3d & getObsUnitVector2() { return m_u_obs2; }
			const Vector3d & getObsErrorVector2() { return m_a_obs2; }
			const double & getObsMinimumDist2() { return m_dist2; }
			const double & getObsActivation2() { return m_obs_activation2; }


			const MatrixXd & getObsJacobian3() { return m_J_obs3; }
			const Vector3d & getObsUnitVector3() { return m_u_obs3; }
			const Vector3d & getObsErrorVector3() { return m_a_obs3; }
			const double & getObsMinimumDist3() { return m_dist3; }
			const double & getObsActivation3() { return m_obs_activation3; }


            void getBasePos(Vector3d &base_p){
                base_pos = base_p;
            }
            void getBaseRot(Matrix3d &base_r){
                base_rot = base_r;
            }
            void getObsPos(Vector3d &obs_p){
                obs_pos = obs_p;
            }
            void getObsPos2(Vector3d &obs_p2){
                obs_pos2 = obs_p2;
            }            
        private:
            std::vector<Vec3f> p[modelnum]; 
            std::vector<Triangle> t[modelnum];
            fcl::Transform3f tf[modelnum];
            fcl::Matrix3f Rot[modelnum];
            Vec3f Trs[modelnum];
            FCL_REAL temp[3];
            fcl::DistanceRes Dis[linkpair];
            fcl::BVHModel<OBBRSS> model[modelnum];
            HQP::robot::RobotModel m_robot;
            bool verbose_[4] ;

            int min_index;
            Vector3d x_avoid;
            MatrixXd Jaco_avoid;
            VectorXd q_;
            VectorXd distance_vec;
			Vector3d link_com[dof];
			Matrix3d link_rot[dof];
			Vector3d link_com_fcl[5];
			Matrix3d link_rot_fcl[5];
			Vector3d joint_pos[dof];
			Vector3d P_a[linkpair];
			Vector3d P_b[linkpair];
            Vector3d com_pos_dis[linkpair];
            
			MatrixXd Jaco[linkpair];
			MatrixXd Jaco_[linkpair];
			MatrixXd Jaco_pos[linkpair];

           MatrixXd m_J_obs, m_J_obs2, m_J_obs3;
          Vector3d m_u_obs, m_u_obs_vel, m_obs_avoid_vec, m_a_obs, m_point;
          Vector3d m_u_obs2, m_u_obs_vel2, m_obs_avoid_vec2, m_a_obs2, m_point2;
          Vector3d m_u_obs3, m_u_obs_vel3, m_obs_avoid_vec3, m_a_obs3, m_point3;
          Vector3d m_f_obs[linkpair];
          double m_f_obs_val[linkpair];

          double m_dist, m_obs_activation;   // shorteds
          double m_dist2, m_obs_activation2; // shorteds
          double m_dist3, m_obs_activation3; // shorteds

         Vector3d base_pos;
         Matrix3d base_rot;
         Vector3d obs_pos, obs_pos2;


          double m_obs_h_value[linkpair];
          double h_value[linkpair];

            Vector3d m_left_pt[2];
          Vector3d m_left_for[2];
          VectorXd m_left_dis[2];


        };
    }   
}




#endif