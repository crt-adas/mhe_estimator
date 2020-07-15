#include <ros/ros.h>
#include "ros_utils.hpp"
#include "kinematics.hpp"
#include "estimators.hpp"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <casadi/casadi.hpp>

#include <mhe_estimator/ArticulatedAngles.h>
#include <mhe_estimator/CanData.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"

//--------------------------| Moving Horizen Window Length |---------------------------// 
const long unsigned int N_mhe = 50;
//-------------------------------------------------------------------------------------// 
/*
namespace mhe_estimator
{
    
  class MheReset
  {
    ::ros::ServiceServer srv_;
    bool resetCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        //rest mhe here,
        
        if(!sawPerceptionMsg)
        {
            sawPerceptionMsg = true;
            qCarEst= qCarLoc;
            qOneTrailerEst = qOneTrailerLoc;  

        }
        
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
            //ROS_INFO_STREAM("reset function has been called");
        }
  };
  
  
}
*/


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
    ::ros::Time lastPerceptionTime;
    const long unsigned int N_mhePlus1 = N_mhe + 1;
    boost::array<Vec4, N_mhePlus1> q4wLoc;        //x y theta beta window
    boost::array<Vec2, N_mhe> control2w;           //control2 window

    boost::array<Vec4, N_mhePlus1> q4wCov;        //state cov window
    boost::array<Vec2, N_mhe> control2wCov;        //control cov window

    boost::array<Vec5, N_mhePlus1> q5wLocTrailer; //x y theta beta window triler =1 
    boost::array<Vec2, N_mhe> control2wTrailer;    //control2 window triler =1 

    boost::array<Vec5, N_mhePlus1> q5wCovTrailer; //state cov window trailer1
    boost::array<Vec2, N_mhe> control2wCovTrailer; //control cov window trailer1 

    Vec4 qLoc;                 //triler == 0 
    Vec2 controls;              //triler == 0 
    Vec4 qEstFirstSample;       //triler == 0 

    Vec4 qCov;               //cov triler == 0 
    Vec4 qCovMhe;
    Vec2 controlsCov;           //cov triler == 0 

    Vec5 qLocTrailer;         //triler == 1 
    Vec2 controlsTrailer;        //triler == 1 
    Vec5 qEstFirstSampleTrailer; //triler == 1 

    Vec5 qCovTrailer;
    Vec5 qCovMheTrailer;         //Cov triler == 1 
    Vec2 controlsCovTrailer;     //cov triler == 1 

    Vec3 q;
    Vec4 qTrailerLoc;
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
    mheSetupCar(solverCar, carParams, mheParams,N_mhe);
    mheSetupTrailer(solverTrailer, carParams, mheParams,N_mhe);
    ROS_INFO_STREAM("Car like solver: "<< solverCar <<"");
    ROS_INFO_STREAM("single wagon like solver: "<< solverTrailer <<"");
    
    casadi::DM argx0 = casadi::SX::zeros((4*(N_mhe + 1))+(2*N_mhe));  // 4 =  n_state    2 = n_control   //triler = 0
    casadi::DM argx0Trailer = casadi::SX::zeros((5*(N_mhe + 1))+(2*N_mhe));  // 4 =  n_state    2 = n_control  //triler =1 
    /*------------------------------------------------------*/

    /*----------------| ROS Subscribers and Publishers  |-----------------------------------------*/

    auto canDataIn = nh.Input<CanData>("/processed_can_data");
    CanData canData;

    auto perceptionPoseCamIn = nh.Input<geometry_msgs::PoseStamped>("/pose_estimator/charger_pose/location_cam");
    auto perceptionPoseGpsIn = nh.Input<geometry_msgs::PoseStamped>("/pose_estimator/charger_pose/location_gps");
    geometry_msgs::PoseStamped perceptionPose;

    auto perceptionTwistOut = nh.Output<geometry_msgs::Twist>("mhe_node/perception/twist");
    geometry_msgs::Twist perceptionTwistData;

    auto perceptionTwistMheOut = nh.Output<geometry_msgs::Twist>("mhe_node/mhe_estimated/twist");
    geometry_msgs::Twist perceptionTwistMheData;

    auto perceptionTwistWeightedOut = nh.Output<geometry_msgs::Twist>("mhe_node/weighted_estimated/twist");
    geometry_msgs::Twist perceptionTwistWeightedData;

    auto articulatedAnglesOut = nh.Output<ArticulatedAngles>("mhe_node/can_data/articulated_angles");
    ArticulatedAngles articulatedAnglesData;

    auto articulatedAnglesMheOut = nh.Output<ArticulatedAngles>("mhe_node/mhe_estimated/articulated_angles");
    ArticulatedAngles articulatedAnglesMheData;

    auto articulatedAnglesWeightedOut = nh.Output<ArticulatedAngles>("mhe_node/weighted_estimated/articulated_angles");
    ArticulatedAngles articulatedAnglesWeightedData;

    auto poseWithCovarianceIn = nh.Input<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/perception_data/pose_with_covariance");
    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceData;
    
    auto poseWithCovarianceMheOut = nh.Output<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/mhe_estimated/pose_with_covariance");
    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceMheData;

    auto poseWithCovarianceWeightedOut = nh.Output<geometry_msgs::PoseWithCovarianceStamped>("mhe_node/weighted_estimated/pose_with_covariance");
    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceWeightedData;

    auto ackermannDriveOut = nh.Output<ackermann_msgs::AckermannDrive>("mhe_node/can_data/ackermann_drive");
    ackermann_msgs::AckermannDrive ackermannDriveData;

    auto ackermannDriveMheOut = nh.Output<ackermann_msgs::AckermannDrive>("mhe_node/mhe_estimated/ackermann_drive");
    ackermann_msgs::AckermannDrive ackermannDriveMheData;
    
    //auto ackermannDriveWeightedOut = nh.Output<ackermann_msgs::AckermannDrive>("mhe_node/weighted_estimated/ackermann_drive");
    //ackermann_msgs::AckermannDrive ackermannDriveWeightedData;
    /*------------------------------------------------------------------------------------------*/

    if (carParams.TrailerNumber == 0)
    {
        ROS_INFO("mhe reset");

        //qCarFirst[0] = qLocMhe[0]; //we can't just assign, vec3!=vec4
        //qCarFirst[1] = qLocMhe[1];
        //qCarFirst[2] = qLocMhe[2];
        //zero the mhe windows
        controls[0] = 0;
        controls[1] = 0;
        for(int i=0; i < N_mhe; i++) 
        {
            control2w[i] = controls;
        }
        for(int i=0; i < (N_mhe + 1); i++) 
        {
            //q4wLoc[i] = qLocMhe;
            q4wCov[i] = {0.0, 0.0, 0.0, 0.0};
        }
    }else if (carParams.TrailerNumber == 1)
    {
        ROS_INFO("mhe reset");
        
        //qOneTrailerFirst[0] = qLocMheTrailer[0];
        //qOneTrailerFirst[1] = qLocMheTrailer[1];
        //qOneTrailerFirst[2] = qLocMheTrailer[2];
        //qOneTrailerFirst[3] = qLocMheTrailer[3];
        //zero the mhe windows
        controlsTrailer[0] = 0;
        controlsTrailer[1] = 0;
        for(int i=0; i < N_mhe; i++) 
        {
            control2wTrailer[i] = controlsTrailer;
        }
        for(int i=0; i < (N_mhe + 1); i++) 
        {
            //q5wLocTrailer[i] = qLocMheTrailer;
            q5wCovTrailer[i] = {0.0, 0.0, 0.0, 0.0, 0.0};
        }
    }else
    {
        ROS_ERROR(" Trailer Number out of range, accepted values: 0, 1");
    }

    
    ROS_INFO_STREAM("Estimator loop rate: "<< mheParams.loopRate <<"");
    Real t = 0;
    ::ros::Rate loop_rate(mheParams.loopRate);
    while(::ros::ok())
    {
        
        //MheReset MheReset(nh);
        
        //*poseWithCovarianceData = poseWithCovarianceIn();
        //*articulatedAnglesData = articulatedAnglesIn();
        //*ackermannDriveData = ackermannDriveIn();
     
        canData = canDataIn(); 
        ackermannDriveData.speed = canData.tachoVelocity;
        ackermannDriveData.steering_angle = canData.steeringAngle;

        if(mheParams.perceptionGPS)
        {
            perceptionPose = perceptionPoseGpsIn(); 
        }else
        {
            perceptionPose = perceptionPoseCamIn();  
        }
         
        auto perceptionTh = tf::getYaw(perceptionPose.pose.orientation);
        ::ros::Duration timeDiff = perceptionPose.header.stamp - lastPerceptionTime ;
        lastPerceptionTime = perceptionPose.header.stamp;                           
        bool isPerceptionPoseFresh = timeDiff.toSec() >= 0.01;

        perceptionTwistData.linear.x = perceptionPose.pose.position.x;// + carParams.L*cos(perceptionTh);
        perceptionTwistData.linear.y = perceptionPose.pose.position.y;// + carParams.L*sin(perceptionTh);
        perceptionTwistData.angular.z = perceptionTh;
        articulatedAnglesData.trailer1 = canData.beta1;
        if(perceptionPose.pose.position.x != 0 && perceptionPose.pose.position.y != 0)
        {
            if (carParams.TrailerNumber == 0)
            {
                
                //Th base on estimator 
                qLoc[1] = perceptionPose.pose.position.x - carParams.L*cos(perceptionTh);
                qLoc[2] = perceptionPose.pose.position.y - carParams.L*sin(perceptionTh);
                qLoc[3] = canData.steeringAngle;
                
                if(mheParams.mheActive)
                {
                    qLoc[0] = continuousAngle(perceptionTh, qCarEstMhe[0]);
                    controls[0] = 0;  //no measured value for dbeta
                    controls[1] = canData.tachoVelocity; //uCar.longitudinalVelocity + noiseArray[4];

                    qCov[0] = 1/mheParams.noiseVarianceTh;         
                    qCov[1] = 1/mheParams.noiseVariancePos;
                    qCov[2] = 1/mheParams.noiseVariancePos;
                    qCov[3] = 1/mheParams.noiseVariancesteering;
                    controlsCov[0] = 0;
                    controlsCov[0] = 1/mheParams.noiseVarianceLinearVel;

                    std::rotate(q4wLoc.begin(), q4wLoc.begin()+1, q4wLoc.end());
                    std::rotate(control2w.begin(), control2w.begin()+1, control2w.end());
                    std::rotate(q4wCov.begin(), q4wCov.begin()+1, q4wCov.end());
                    std::rotate(control2wCov.begin(), control2wCov.begin()+1, control2wCov.end());
            
                    q4wCov[N_mhe] = {0.0, 0.0, 0.0, 0.0};
                    ::ros::Duration sampleDiff = ::ros::Time::now() - perceptionPose.header.stamp;
                    int sampleNumber = floor(sampleDiff.toSec()/(1.0/mheParams.loopRate));
                    if (sampleNumber > N_mhe)
                    {
                        ROS_WARN_STREAM("WARNING: localization delay = "<< sampleNumber<<" sample" );
                    }
                    if(sampleNumber >=0  && sampleNumber < N_mhe)
                    {
                        ROS_INFO_STREAM("SampleIndex: "<<sampleNumber <<" ");
                        q4wCov[N_mhe - sampleNumber] = qCov;
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

                    ctrlEstMhe[0] = resx[(4*(N_mhe+1))+(4*N_mhe)-4]; //dbeta
                    ctrlEstMhe[1] = resx[(4*(N_mhe+1))+(4*N_mhe)-3]; //u2

                    qCovMhe[0] = resx[(4*(N_mhe+1))+(4*N_mhe)+(4*(N_mhe+1))-4];//theta
                    qCovMhe[1] = resx[(4*(N_mhe+1))+(4*N_mhe)+(4*(N_mhe+1))-3];//x 
                    qCovMhe[2] = resx[(4*(N_mhe+1))+(4*N_mhe)+(4*(N_mhe+1))-2];//y
                    qCovMhe[3] = resx[(4*(N_mhe+1))+(4*N_mhe)+(4*(N_mhe+1))-1];//beta0
                    
                    poseWithCovarianceMheData.pose.pose.orientation = tf::createQuaternionMsgFromYaw(qCarEstMhe[0]);
                    poseWithCovarianceMheData.pose.pose.position.x = qCarEstMhe[1] + carParams.L*cos(qCarEstMhe[0]);
                    poseWithCovarianceMheData.pose.pose.position.y = qCarEstMhe[2] + carParams.L*sin(qCarEstMhe[0]);

                    poseWithCovarianceMheData.pose.covariance.elems[0] = qCovMhe[1];
                    poseWithCovarianceMheData.pose.covariance.elems[7] = qCovMhe[2];
                    poseWithCovarianceMheData.pose.covariance.elems[35] = qCovMhe[0];
                    poseWithCovarianceMheOut(poseWithCovarianceMheData);

                    ackermannDriveMheData.steering_angle = qCarEstMhe[3];
                    ackermannDriveMheData.speed = ctrlEstMhe[1];
                    ackermannDriveMheData.steering_angle_velocity = ctrlEstMhe[0];
                    ackermannDriveMheOut(ackermannDriveMheData);

                    perceptionTwistMheData.angular.z = qCarEstMhe[0];
                    perceptionTwistMheData.linear.x = qCarEstMhe[1] + carParams.L*cos(qCarEstMhe[0]);
                    perceptionTwistMheData.linear.y = qCarEstMhe[2] + carParams.L*sin(qCarEstMhe[0]);
                    perceptionTwistMheOut(perceptionTwistMheData);

                }
                if(mheParams.WeightedActive)
                {
                    qLoc[0] = continuousAngle(perceptionTh, qCarEst[0]);
                    q = {qLoc[0],qLoc[1],qLoc[2]};
                    auto simFunc = [&](const Vec3& q, Vec3& dq, const double t)
                    {
                        //if(!carParams.moveGuidancePoint)
                            dq = RDCarKinematicsGPRear(carParams, q, ackermannDriveData);
                        //else
                        //  dq = RDCarKinematicsGPFront(carParams, q, ackermannDriveData);
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
                        estimateEst(qCarEst, q, qCarPred ,mheParams);  
                    }
                    poseWithCovarianceWeightedData.pose.pose.orientation = tf::createQuaternionMsgFromYaw(qCarEst[0]);
                    poseWithCovarianceWeightedData.pose.pose.position.x = qCarEst[1] + carParams.L*cos(qCarEst[0]);
                    poseWithCovarianceWeightedData.pose.pose.position.y = qCarEst[2] + carParams.L*sin(qCarEst[0]);
                    poseWithCovarianceWeightedData.header.stamp = ::ros::Time::now();
                    poseWithCovarianceWeightedOut(poseWithCovarianceWeightedData);

                    perceptionTwistWeightedData.angular.z = qCarEst[0];
                    perceptionTwistWeightedData.linear.x = qCarEst[1] + carParams.L*cos(qCarEst[0]);
                    perceptionTwistWeightedData.linear.y = qCarEst[2] + carParams.L*sin(qCarEst[0]);
                    perceptionTwistWeightedOut(perceptionTwistWeightedData);
                }
            }else if (carParams.TrailerNumber == 1)
            {
                
                qLocTrailer[0] = canData.beta1;
                //Th base on estimator is different
                qLocTrailer[2] = perceptionPose.pose.position.x - carParams.L*cos(perceptionTh);
                qLocTrailer[3] = perceptionPose.pose.position.y - carParams.L*sin(perceptionTh);
                qLocTrailer[4] = canData.steeringAngle;
                

                if(mheParams.mheActive)
                {
                    qLocTrailer[1] = continuousAngle(perceptionTh, qMheTrailer[1]); 
                    controlsTrailer[0] = 0; //no measured value for dbeta
                    controlsTrailer[1] = canData.tachoVelocity; //uCar.longitudinalVelocity + noiseArray[4];
                
                    qCovTrailer[0] = 1/mheParams.noiseVarianceTrailer1;
                    qCovTrailer[1] = 1/mheParams.noiseVarianceTh;
                    qCovTrailer[2] = 1/mheParams.noiseVariancePos;
                    qCovTrailer[3] = 1/mheParams.noiseVariancePos;
                    qCovTrailer[4] = 1/mheParams.noiseVariancesteering;
            
                    controlsCovTrailer[0] = 0;
                    controlsCovTrailer[0] = 1/mheParams.noiseVarianceLinearVel;
                
                    std::rotate(q5wLocTrailer.begin(), q5wLocTrailer.begin()+1, q5wLocTrailer.end());
                    std::rotate(control2wTrailer.begin(), control2wTrailer.begin()+1, control2wTrailer.end());
                    std::rotate(q5wCovTrailer.begin(), q5wCovTrailer.begin()+1, q5wCovTrailer.end());
                    std::rotate(control2wCovTrailer.begin(), control2wCovTrailer.begin()+1, control2wCovTrailer.end());
                    //q5wCovTrailer[N_mhe] = {0.0, 0.0, 0.0, 0.0, 0.0};
                
                    ::ros::Duration sampleDiff = ::ros::Time::now() - perceptionPose.header.stamp;
                    int sampleNumber = floor(sampleDiff.toSec()/(1.0/mheParams.loopRate));
                    if (sampleNumber > N_mhe)
                    {
                        ROS_WARN_STREAM("WARNING: localization delay = "<< sampleNumber<<" sample" );
                    }
                    if(sampleNumber >=0  && sampleNumber < N_mhe)
                    {
                        ROS_INFO_STREAM("SampleIndex: "<<sampleNumber <<" ");
                        //q5wCovTrailer[N_mhe - sampleNumber] = qCovTrailer;
                        //q5wLocTrailer[N_mhe - sampleNumber] = qLocTrailer;
                    }
                    q5wCovTrailer[N_mhe] = qCovTrailer;
                    q5wLocTrailer[N_mhe] = qLocTrailer;
                    q5wLocTrailer[0] = qEstFirstSampleTrailer;  
                    control2wTrailer[N_mhe-1] = controlsTrailer; 
                    control2wCovTrailer[N_mhe-1] = controlsCovTrailer;
                
                    estimateMheTrailer(argx0Trailer, q5wLocTrailer, control2wTrailer, q5wCovTrailer, control2wCovTrailer, carParams, mheParams,solverTrailer);
                        
                    std::vector<double> resxTrailer = std::vector<double>(argx0Trailer);
                    qMheTrailer[0] = resxTrailer[(5*(N_mhe+1))-5]; //beta1                return estimated 4 = n_states
                    qMheTrailer[1] = resxTrailer[(5*(N_mhe+1))-4]; //theta
                    qMheTrailer[2] = resxTrailer[(5*(N_mhe+1))-3]; //x
                    qMheTrailer[3] = resxTrailer[(5*(N_mhe+1))-2]; //y
                    qMheTrailer[4] = resxTrailer[(5*(N_mhe+1))-1]; //beta0

                    qEstFirstSampleTrailer[0] = resxTrailer[0]; //beta1                return estimated 4 = n_states
                    qEstFirstSampleTrailer[1] = resxTrailer[1]; //theta
                    qEstFirstSampleTrailer[2] = resxTrailer[2]; //x
                    qEstFirstSampleTrailer[3] = resxTrailer[3]; //y
                    qEstFirstSampleTrailer[4] = resxTrailer[4]; //beta0

                    ctrMheTrailer[0] = resxTrailer[(5*(N_mhe+1))+(5*N_mhe)-5]; //dbeta
                    ctrMheTrailer[1] = resxTrailer[(5*(N_mhe+1))+(5*N_mhe)-4]; //u2
                    
                    qCovMheTrailer[0] = resxTrailer[(5*(N_mhe+1))+(5*N_mhe)+(5*(N_mhe+1))-5]; //beta1
                    qCovMheTrailer[1] = resxTrailer[(5*(N_mhe+1))+(5*N_mhe)+(5*(N_mhe+1))-4]; //theta
                    qCovMheTrailer[2] = resxTrailer[(5*(N_mhe+1))+(5*N_mhe)+(5*(N_mhe+1))-3]; //x
                    qCovMheTrailer[3] = resxTrailer[(5*(N_mhe+1))+(5*N_mhe)+(5*(N_mhe+1))-2]; //y
                    qCovMheTrailer[4] = resxTrailer[(5*(N_mhe+1))+(5*N_mhe)+(5*(N_mhe+1))-1]; //beta0

                    poseWithCovarianceMheData.pose.pose.orientation = tf::createQuaternionMsgFromYaw(qMheTrailer[1]);
                    poseWithCovarianceMheData.pose.pose.position.x = qMheTrailer[2] + carParams.L*cos(qMheTrailer[1]);
                    poseWithCovarianceMheData.pose.pose.position.y = qMheTrailer[3] + carParams.L*sin(qMheTrailer[1]);

                    poseWithCovarianceMheData.pose.covariance.elems[0] = qCovMheTrailer[2];
                    poseWithCovarianceMheData.pose.covariance.elems[7] = qCovMheTrailer[3];
                    poseWithCovarianceMheData.pose.covariance.elems[35] = qCovMheTrailer[1];
                    poseWithCovarianceMheOut(poseWithCovarianceMheData);

                    ackermannDriveMheData.steering_angle = qMheTrailer[4];
                    ackermannDriveMheData.speed = ctrMheTrailer[1];
                    ackermannDriveMheData.steering_angle_velocity = ctrMheTrailer[0];
                    ackermannDriveMheOut(ackermannDriveMheData);

                    perceptionTwistMheData.angular.z = qMheTrailer[1];
                    perceptionTwistMheData.linear.x = qMheTrailer[2] + carParams.L*cos(qMheTrailer[1]);
                    perceptionTwistMheData.linear.y = qMheTrailer[3] + carParams.L*sin(qMheTrailer[1]);
                    perceptionTwistMheOut(perceptionTwistMheData);

                    articulatedAnglesMheData.trailer1 = qMheTrailer[0];
                    articulatedAnglesMheOut(articulatedAnglesMheData);
                
                
                }   
                if(mheParams.WeightedActive)
                {
                    
                    qLocTrailer[1] = continuousAngle(perceptionTh, qOneTrailerEst[1]);
                    qTrailerLoc = {qLocTrailer[0],qLocTrailer[1],qLocTrailer[2],qLocTrailer[3] };
                    auto simFunc = [&](const Vec4& qTrailer, Vec4& dq, const double t)
                    {
                        if(!carParams.moveGuidancePoint)
                            dq = OneTrailerKinematicsGPRear(carParams, qTrailer, ackermannDriveData);
                        else
                            dq = OneTrailerKinematicsGPFront(carParams, qTrailer, ackermannDriveData);
                    };
                        
                    Vec4 qOneTrailerPred = qOneTrailerEst;
                    boost::numeric::odeint::integrate(simFunc, qOneTrailerPred, 0.0, 1.0/(Real)mheParams.loopRate, 1.0/(Real)mheParams.loopRate);
                    
                    if(!isPerceptionPoseFresh)
                    {
                        qOneTrailerEst = qOneTrailerPred;
                    }
                    else
                    {
                        ROS_INFO("fresh gps pose");
                        estimateEstTrailer(qOneTrailerEst, qTrailerLoc, qOneTrailerPred, mheParams);
                        //betaEst = estParams.weightBeta*betaPred + (1-estParams.weightBeta)*(beta);
                    }
                    poseWithCovarianceWeightedData.pose.pose.orientation = tf::createQuaternionMsgFromYaw(qOneTrailerEst[1]);
                    poseWithCovarianceWeightedData.pose.pose.position.x = qOneTrailerEst[2]  + carParams.L*cos(qOneTrailerEst[1]);
                    poseWithCovarianceWeightedData.pose.pose.position.y = qOneTrailerEst[3]  + carParams.L*sin(qOneTrailerEst[1]);
                    poseWithCovarianceWeightedData.header.stamp = ::ros::Time::now();
                    poseWithCovarianceWeightedOut(poseWithCovarianceWeightedData);
                    

                    articulatedAnglesWeightedData.trailer1 = qOneTrailerEst[0];
                    articulatedAnglesWeightedOut(articulatedAnglesWeightedData);

                    perceptionTwistWeightedData.angular.z = qOneTrailerEst[1];
                    perceptionTwistWeightedData.linear.x = qOneTrailerEst[2] + carParams.L*cos(qOneTrailerEst[1]);
                    perceptionTwistWeightedData.linear.y = qOneTrailerEst[3] + carParams.L*sin(qOneTrailerEst[1]);
                    perceptionTwistWeightedOut(perceptionTwistWeightedData);



                    

                }
            }else
            {
                ROS_ERROR(" Trailer Number out of range, accepted values: 0, 1");
            }
            perceptionTwistOut(perceptionTwistData);
            articulatedAnglesOut(articulatedAnglesData);
            ackermannDriveOut(ackermannDriveData);
        }
        
        t += 1/((Real) mheParams.loopRate);
        ::ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
