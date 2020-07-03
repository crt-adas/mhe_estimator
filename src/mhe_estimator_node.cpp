#include "ros_utils.hpp"
#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include <mhe_estimator/MheParams.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace mhe
{

  typedef double Real;
  

  struct CarParams
  {
      bool moveGuidancePoint;    
      Real L, L1, Lh1, L2, Lh2;
      Real steeringLimit, trailer1Limit, trailer2Limit, LinearVelLimit, SteeringVelLimit;
      Real upperTh, lowerTh, upperX, lowerX, upperY, lowerY;
      Real TrailerNumber;
  };

  struct MheParam
  {
      bool mheActive, WeightedActive, covarianceFromTopicStamp;
      Real N_mhe, loopRate;   
      Real noiseVariancePos, noiseVarianceTh, noiseVariancesteering, noiseVarianceTrailer1;
      Real noiseVarianceTrailer2, noiseVarianceLinearVel, noiseVarianceSteeringVel;
      Real WeightPos, WeightTh, WeightSteering, WeightTrailer1, WeightTrailer2;  
  };

  template <class T>
  void singleParamIn(T& ParamVar,std::string& paramName,ast::ros::NodeHandle& nh)
  {
    if (nh.hasParam(paramName))
    {
      nh.getParam(paramName,ParamVar);
      ROS_INFO_STREAM(""<<paramName<<" = "<<ParamVar<<"");
    }else
    {
      ROS_WARN_STREAM("parameter: " << paramName << " could not be found");
    }
  }

  void ParamsIn(mhe::CarParams& carParams,mhe::MheParam& mheParam,ast::ros::NodeHandle& nh)
  {
      std::string paramName;

      paramName = "/mhe_estimator/mheParam/mheActive";
      singleParamIn(mheParam.mheActive,paramName,nh);
      paramName = "/mhe_estimator/mheParam/WeightedActive";
      singleParamIn(mheParam.WeightedActive,paramName,nh);
      paramName = "/mhe_estimator/mheParam/windowLength";
      singleParamIn(mheParam.N_mhe,paramName,nh);
      paramName = "/mhe_estimator/mheParam/loopRate";
      singleParamIn(mheParam.loopRate,paramName,nh);
      paramName = "/mhe_estimator/mheParam/covarianceFromTopicStamp";
      singleParamIn(mheParam.covarianceFromTopicStamp,paramName,nh);
      paramName = "/mhe_estimator/mheParam/noiseVariancePos";
      singleParamIn(mheParam.noiseVariancePos,paramName,nh);
      paramName = "/mhe_estimator/mheParam/noiseVarianceTh";
      singleParamIn(mheParam.noiseVarianceTh,paramName,nh);
      paramName = "/mhe_estimator/mheParam/noiseVariancesteering";
      singleParamIn(mheParam.noiseVariancesteering,paramName,nh);
      paramName = "/mhe_estimator/mheParam/noiseVarianceTrailer1";
      singleParamIn(mheParam.noiseVarianceTrailer1,paramName,nh);
      paramName = "/mhe_estimator/mheParam/noiseVarianceTrailer2";
      singleParamIn(mheParam.noiseVarianceTrailer2,paramName,nh);
      paramName = "/mhe_estimator/mheParam/noiseVarianceLinearVel";
      singleParamIn(mheParam.noiseVarianceLinearVel,paramName,nh);
      paramName = "/mhe_estimator/mheParam/noiseVarianceSteeringVel";
      singleParamIn(mheParam.noiseVarianceSteeringVel,paramName,nh);
      paramName = "/mhe_estimator/mheParam/windowLength";
      singleParamIn(mheParam.WeightPos,paramName,nh);
      paramName = "/mhe_estimator/mheParam/loopRate";
      singleParamIn(mheParam.WeightTh,paramName,nh);
      paramName = "/mhe_estimator/mheParam/loopRate";
      singleParamIn(mheParam.WeightSteering,paramName,nh);
      paramName = "/mhe_estimator/mheParam/loopRate";
      singleParamIn(mheParam.WeightTrailer1,paramName,nh);
      paramName = "/mhe_estimator/mheParam/loopRate";
      singleParamIn(mheParam.WeightTrailer2,paramName,nh);
      paramName = "/mhe_estimator/carParam/WagonNumbers";
      singleParamIn(carParams.TrailerNumber,paramName,nh);
      paramName = "/mhe_estimator/carParam/L";
      singleParamIn(carParams.L,paramName,nh);
      paramName = "/mhe_estimator/carParam/L1";
      singleParamIn(carParams.L1,paramName,nh);
      paramName = "/mhe_estimator/carParam/Lh1";
      singleParamIn(carParams.Lh1,paramName,nh);
      paramName = "/mhe_estimator/carParam/L2";
      singleParamIn(carParams.L2,paramName,nh);
      paramName = "/mhe_estimator/carParam/Lh2";
      singleParamIn(carParams.Lh2,paramName,nh);
      paramName = "/mhe_estimator/carParam/steeringLimit";
      singleParamIn(carParams.steeringLimit,paramName,nh);
      paramName = "/mhe_estimator/carParam/trailer1Limit";
      singleParamIn(carParams.trailer1Limit,paramName,nh);
      paramName = "/mhe_estimator/carParam/trailer2Limit";
      singleParamIn(carParams.trailer2Limit,paramName,nh);
      paramName = "/mhe_estimator/carParam/LinearVelLimit";
      singleParamIn(carParams.LinearVelLimit,paramName,nh);
      paramName = "/mhe_estimator/carParam/SteeringVelLimit";
      singleParamIn(carParams.SteeringVelLimit,paramName,nh);
      paramName = "/mhe_estimator/carParam/moveGuidancePoint";
      singleParamIn(carParams.moveGuidancePoint,paramName,nh);
      paramName = "/mhe_estimator/carParam/upperTh";
      singleParamIn(carParams.upperTh,paramName,nh);
      paramName = "/mhe_estimator/carParam/lowerTh";
      singleParamIn(carParams.lowerTh,paramName,nh);
      paramName = "/mhe_estimator/carParam/upperX";
      singleParamIn(carParams.upperX,paramName,nh);
      paramName = "/mhe_estimator/carParam/lowerX";
      singleParamIn(carParams.lowerX,paramName,nh);
      paramName = "/mhe_estimator/carParam/upperY";
      singleParamIn(carParams.upperY,paramName,nh);
      paramName = "/mhe_estimator/carParam/lowerY";
      singleParamIn(carParams.lowerY,paramName,nh);
    
      ROS_INFO_STREAM("End of receiving parameters");

  }

}


int main(int argc, char **argv)
{
  using namespace mhe;
  using namespace ast;
  using namespace ast::ros;
  CarParams carParams;
  MheParam mheParam;
  
  ::ros::init(argc, argv, "mhe_estimator_node");
  NodeHandle nh;
  ParamsIn(carParams, mheParam, nh);

  
  ::ros::Rate loop_rate(10);
  
  while (::ros::ok())
  {

    ROS_INFO_STREAM("params and back");
    
  
    ::ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
