#include <ros/ros.h>
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include "estimators.hpp"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <casadi/casadi.hpp>

#include <mhe_estimator/ArticulatedAngles.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"

namespace mhe_estimator
{

  class MheReset
  {
    
    ::ros::ServiceServer srv_;
    bool resetCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        //rest mhe here,
        /*
         if(!sawPerceptionMsg)
        {
            sawPerceptionMsg = true;
            qCarEst= qCarLoc;
            dockingLcGenerator.computeParams(beta, qCarEst[0], qCarEst[1], qCarEst[2], dockingSafetyOffset);
        }
        */
        res.success = true;
        std::string str("MHE window reset successfully.");
        ROS_INFO_STREAM("MHE window reset successfully");
        res.message = str;
        return true;
    }
    public:
        MheReset (::ros::NodeHandle& nh)
        : srv_ (nh.advertiseService ("mhe_estimator/reset", &MheReset::resetCallBack, this))
        {
            ROS_INFO_STREAM("reset has been called");
        }
  };
  
}



int main(int argc, char **argv)
{
    using namespace mhe_estimator;
    using namespace ast;
    using namespace ast::ros;
    ::ros::init(argc, argv, "mhe_estimator_node");
    NodeHandle nh;
    
    /*----------------| Get Params |----------------*/
    CarParams carParams;
    MheParams mheParams;
    ParamsIn(carParams, mheParams, nh);
    /*---------------------------------------------*/

    /*------------| Global Var and Obj |-----------*/
    
    //size_t N_mhe = mheParams.N_mhe;
    //size_t N_mhePlusOne = N_mhe+1;
    ::ros::Time lastPerceptionTime;

    static const long unsigned int N_mhe = 200 ;
    boost::array<Vec4, 201> q4wLoc;        //x y theta beta window
    boost::array<Vec2, N_mhe> control2w;           //control2 window

    boost::array<Vec4, 201> q4wCov;        //state cov window
    boost::array<Vec2, 200> control2wCov;        //control cov window

    boost::array<Vec5, 201> q5wLocTrailer; //x y theta beta window triler =1 
    boost::array<Vec2, 200> control2wTrailer;    //control2 window triler =1 

    boost::array<Vec5, 201> q5wCovTrailer; //state cov window trailer1
    boost::array<Vec2, 200> control2wCovTrailer; //control cov window trailer1 

    Vec4 qLoc;               //triler == 0 
    Vec2 controls;              //triler == 0 
    Vec4 qEstFirstSample;       //triler == 0 

    Vec4 qCovMhe;               //cov triler == 0 
    Vec2 controlsCov;           //cov triler == 0 

    Vec5 qLocMheTrailer;         //triler == 1 
    Vec2 controlsTrailer;        //triler == 1 
    Vec5 qEstFirstSampleTrailer; //triler == 1 

    Vec5 qCovMheTrailer;         //Cov triler == 1 
    Vec2 controlsCovTrailer;     //cov triler == 1 
    Vec3 q;
    Vec3 qCarEst;
    Vec3 qCarFirst;
    Vec4 qOneTrailerFirst;
    Vec4 qCarEstMhe;
    Vec2 ctrlEstMhe;
    Vec5 qMheTrailer;
    Vec2 ctrMheTrailer;
    Vec4 qOneTrailerEst;
    Vec4 qOneTrailerLoc;
    /*---------------------------------------------*/



    /*----------------| Get Casadi Solver  |----------------*/
    casadi::Function solverCar;
    casadi::Function solverTrailer;
    ROS_INFO_STREAM("Creating CasADi Solvers");
    mheSetupCar(solverCar, carParams, mheParams);
    mheSetupTrailer(solverTrailer, carParams, mheParams);
    ROS_INFO_STREAM("Car like solver: "<< solverCar <<"");
    ROS_INFO_STREAM("single wagon like solver: "<< solverTrailer <<"");
    
    casadi::DM argx0 = casadi::SX::zeros((4*(N_mhe + 1))+(2*N_mhe));  // 4 =  n_state    2 = n_control   //triler = 0
    casadi::DM argx0Trailer = casadi::SX::zeros((5*(N_mhe + 1))+(2*N_mhe));  // 4 =  n_state    2 = n_control  //triler =1 
    /*------------------------------------------------------*/

    /*----------------| ROS Subscribers and Publishers  |-----------------------------------------*/
    auto articulatedAnglesIn = nh.Input<ArticulatedAngles>("mhe_node/can_data/articulated_angles");
    boost::shared_ptr<ArticulatedAngles> articulatedAnglesData(new ArticulatedAngles());

    auto articulatedAnglesMheOut = nh.Output<ArticulatedAngles>("mhe_node/mhe_estimated/articulated_angles");
    boost::shared_ptr<ArticulatedAngles> articulatedAnglesMheData(new ArticulatedAngles());

    auto articulatedAnglesWeightedOut = nh.Output<ArticulatedAngles>("mhe_node/weighted_estimated/articulated_angles");
    boost::shared_ptr<ArticulatedAngles> articulatedAnglesWeightedData(new ArticulatedAngles());

    auto poseWithCovarianceIn = nh.Input<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/perception_data/pose_with_covariance");
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> poseWithCovarianceData(new geometry_msgs::PoseWithCovarianceStamped());
    
    auto poseWithCovarianceMheOut = nh.Output<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/mhe_estimated/pose_with_covariance");
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> poseWithCovarianceMheData(new geometry_msgs::PoseWithCovarianceStamped());

    auto poseWithCovarianceWeightedOut = nh.Output<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/weighted_estimated/pose_with_covariance");
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> poseWithCovarianceWeightedData(new geometry_msgs::PoseWithCovarianceStamped());

    auto ackermannDriveIn = nh.Input<ackermann_msgs::AckermannDrive>("mhe_node/can_data/ackermann_drive");
    boost::shared_ptr<ackermann_msgs::AckermannDrive> ackermannDriveData(new ackermann_msgs::AckermannDrive());

    auto ackermannDriveMheOut = nh.Output<ackermann_msgs::AckermannDrive>("mhe_node/mhe_estimated/ackermann_drive");
    boost::shared_ptr<ackermann_msgs::AckermannDrive> ackermannDriveMheData(new ackermann_msgs::AckermannDrive());
    
    auto ackermannDriveWeightedOut = nh.Output<ackermann_msgs::AckermannDrive>("mhe_node/weighted_estimated/ackermann_drive");
    boost::shared_ptr<ackermann_msgs::AckermannDrive> ackermannDriveWeightedData(new ackermann_msgs::AckermannDrive());
    /*------------------------------------------------------------------------------------------*/



    ::ros::Rate loop_rate(mheParams.loopRate);
    ROS_INFO_STREAM("Estimator loop rate: "<< mheParams.loopRate <<"");
    Real t = 0;
    while (::ros::ok())
    {
        MheReset MheReset(nh);
        
        *poseWithCovarianceData = poseWithCovarianceIn();
        auto perceptionTh = tf::getYaw(poseWithCovarianceData->pose.pose.orientation);
        ::ros::Duration timeDiff = poseWithCovarianceData->header.stamp - lastPerceptionTime ;
        lastPerceptionTime = poseWithCovarianceData->header.stamp;                           
        bool isPerceptionPoseFresh = timeDiff.toSec() >= 0.01;

        *articulatedAnglesData = articulatedAnglesIn();
        *ackermannDriveData = ackermannDriveIn();
     


        if (carParams.TrailerNumber == 0)
        {

            if(mheParams.mheActive)
            {
                qLoc[0] = continuousAngle(perceptionTh, qCarEstMhe[0]);
                qLoc[1] = poseWithCovarianceData->pose.pose.position.x;
                qLoc[2] = poseWithCovarianceData->pose.pose.position.y;
                qLoc[3] = ackermannDriveData->steering_angle;

                controls[0] = 0;  //no measured value for dbeta
                controls[1] = ackermannDriveData->speed; //uCar.longitudinalVelocity + noiseArray[4];

                qCovMhe[0] = poseWithCovarianceData->pose.covariance.elems[36];         
                qCovMhe[1] = poseWithCovarianceData->pose.covariance.elems[0];
                qCovMhe[2] = poseWithCovarianceData->pose.covariance.elems[7];
                qCovMhe[3] = mheParams.noiseVariancesteering;
                controlsCov[0] = 0;
                controlsCov[0] = mheParams.noiseVarianceLinearVel;

                std::rotate(q4wLoc.begin(), q4wLoc.begin()+1, q4wLoc.end());
                std::rotate(control2w.begin(), control2w.begin()+1, control2w.end());
                std::rotate(q4wCov.begin(), q4wCov.begin()+1, q4wCov.end());
                std::rotate(control2wCov.begin(), control2wCov.begin()+1, control2wCov.end());

                q4wCov[N_mhe] = {0.0, 0.0, 0.0, 0.0};
                ::ros::Duration sampleDiff = ::ros::Time::now() - poseWithCovarianceData->header.stamp;
                int sampleNumber = floor(sampleDiff.toSec()/(1.0/mheParams.loopRate));
                if(sampleNumber >=0  && sampleNumber < N_mhe)
                {
                    q4wCov[N_mhe - sampleNumber] = qCovMhe;
                    q4wLoc[N_mhe - sampleNumber] = qLoc;
                }

                q4wLoc[0] = qEstFirstSample; 
                control2w[N_mhe-1] = controls; //control window is one element shorter
                control2wCov[N_mhe-1] = controlsCov;
                estimateMhe(argx0, q4wLoc, control2w, q4wCov, control2wCov, carParams, mheParams, solverCar);
                std::vector<double> resx = std::vector<double>(argx0);
                qCarEstMhe[0] = resx[(4*(N_mhe+1))-4]; //theta    return estimated 4 = n_states
                qCarEstMhe[1] = resx[(4*(N_mhe+1))-3]; //x 
                qCarEstMhe[2] = resx[(4*(N_mhe+1))-2]; //y
                qCarEstMhe[3] = resx[(4*(N_mhe+1))-1]; //beta0
                
                qEstFirstSample[0] = resx[0]; //theta return estimated 4 = n_states
                qEstFirstSample[1] = resx[1]; //x
                qEstFirstSample[2] = resx[2]; //y
                qEstFirstSample[3] = resx[3]; //beta0

                ctrlEstMhe[0] = resx[(4*(N_mhe+1))+(2*N_mhe)-2]; //dbeta
                ctrlEstMhe[1] = resx[(4*(N_mhe+1))+(2*N_mhe)-1]; //u2
                
                poseWithCovarianceMheData->pose.pose.orientation = tf::createQuaternionMsgFromYaw(qCarEstMhe[0]);
                poseWithCovarianceMheData->pose.pose.position.x = qCarEstMhe[1];
                poseWithCovarianceMheData->pose.pose.position.y = qCarEstMhe[2];
                poseWithCovarianceMheOut(poseWithCovarianceMheData);

                ackermannDriveMheData->steering_angle = qCarEstMhe[3];
                ackermannDriveMheData->speed = ctrlEstMhe[1];
                ackermannDriveMheData->steering_angle_velocity = ctrlEstMhe[0];
                ackermannDriveMheOut(ackermannDriveMheData);
            }
            if(mheParams.WeightedActive)
            {
                q = {qLoc[1],qLoc[2],qLoc[3]};
                auto simFunc = [&](const Vec3& q, Vec3& dq, const double t)
                {
                    //if(!carParams.moveGuidancePoint)
                        dq = RDCarKinematicsGPRear(carParams, q, *ackermannDriveMheData);
                    //else
                    //  dq = RDCarKinematicsGPFront(carParams, q, uCar);
                };
                Vec3 qCarPred = qCarEst;
                boost::numeric::odeint::integrate(simFunc, qCarPred, 0.0, 1.0/(Real)mheParams.loopRate, 1.0/(Real)mheParams.loopRate);

                if(!isPerceptionPoseFresh)
                {
                    qCarEst = qCarPred;
                }
                else
                {
                    ROS_INFO("fresh gps pose");
                    //estimateEst(qCarEst, qCarLoc, qCarPred, carParams.trailersNumber,estParams);                            
                   
                }
            }
        }else if (carParams.TrailerNumber == 1)
        {

            //estimateMheTrailer(argx0Trailer,qMheTrailer, ctrMheTrailer, q5wLocTrailer, control2wTrailer, q5wCovTrailer, control2wCovTrailer, carParams, qFinal, mheParams,solverTrailer);
        }else
        {
            ROS_ERROR(" Trailer Number out of range, accepted values: 1, 2");
        }

        t += 1/((Real) mheParams.loopRate);
        ROS_INFO_STREAM("I'm spinning");
        ::ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
