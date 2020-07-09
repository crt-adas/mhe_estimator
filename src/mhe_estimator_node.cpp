#include <ros/ros.h>
#include "ros_utils.hpp"
#include <boost/array.hpp>

#include <casadi/casadi.hpp>

#include <mhe_estimator/ArticulatedAngles.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"

namespace mhe_estimator
{

    typedef double Real;
    typedef boost::array<Real, 6> Vec6;
    typedef boost::array<Real, 5> Vec5;
    typedef boost::array<Real, 4> Vec4;
    typedef boost::array<Real, 3> Vec3;
    typedef boost::array<Real, 2> Vec2;

    struct CarParams
    {
        bool moveGuidancePoint;    
        Real L, L1, Lh1, L2, Lh2;
        Real steeringLimit, trailer1Limit, trailer2Limit, LinearVelLimit, SteeringVelLimit;
        Real upperTh, lowerTh, upperX, lowerX, upperY, lowerY;
        Real TrailerNumber;
    };

    struct MheParams
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

    void ParamsIn(mhe_estimator::CarParams& carParams,mhe_estimator::MheParams& mheParams,ast::ros::NodeHandle& nh)
    {
        std::string paramName;

        paramName = "/mhe_estimator/mheParam/mheActive";
        singleParamIn(mheParams.mheActive,paramName,nh);
        paramName = "/mhe_estimator/mheParam/WeightedActive";
        singleParamIn(mheParams.WeightedActive,paramName,nh);
        paramName = "/mhe_estimator/mheParam/windowLength";
        singleParamIn(mheParams.N_mhe,paramName,nh);
        paramName = "/mhe_estimator/mheParam/loopRate";
        singleParamIn(mheParams.loopRate,paramName,nh);
        paramName = "/mhe_estimator/mheParam/covarianceFromTopicStamp";
        singleParamIn(mheParams.covarianceFromTopicStamp,paramName,nh);
        paramName = "/mhe_estimator/mheParam/noiseVariancePos";
        singleParamIn(mheParams.noiseVariancePos,paramName,nh);
        paramName = "/mhe_estimator/mheParam/noiseVarianceTh";
        singleParamIn(mheParams.noiseVarianceTh,paramName,nh);
        paramName = "/mhe_estimator/mheParam/noiseVariancesteering";
        singleParamIn(mheParams.noiseVariancesteering,paramName,nh);
        paramName = "/mhe_estimator/mheParam/noiseVarianceTrailer1";
        singleParamIn(mheParams.noiseVarianceTrailer1,paramName,nh);
        paramName = "/mhe_estimator/mheParam/noiseVarianceTrailer2";
        singleParamIn(mheParams.noiseVarianceTrailer2,paramName,nh);
        paramName = "/mhe_estimator/mheParam/noiseVarianceLinearVel";
        singleParamIn(mheParams.noiseVarianceLinearVel,paramName,nh);
        paramName = "/mhe_estimator/mheParam/noiseVarianceSteeringVel";
        singleParamIn(mheParams.noiseVarianceSteeringVel,paramName,nh);
        paramName = "/mhe_estimator/mheParam/windowLength";
        singleParamIn(mheParams.WeightPos,paramName,nh);
        paramName = "/mhe_estimator/mheParam/WeightTh";
        singleParamIn(mheParams.WeightTh,paramName,nh);
        paramName = "/mhe_estimator/mheParam/WeightSteering";
        singleParamIn(mheParams.WeightSteering,paramName,nh);
        paramName = "/mhe_estimator/mheParam/WeightTrailer1";
        singleParamIn(mheParams.WeightTrailer1,paramName,nh);
        paramName = "/mhe_estimator/mheParam/WeightTrailer2";
        singleParamIn(mheParams.WeightTrailer2,paramName,nh);
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

    void mheSetupCar(casadi::Function& solver, const mhe_estimator::CarParams& params, const mhe_estimator::MheParams& mheParams)
    {
        using namespace casadi;
        double A = mheParams.N_mhe;
        double B = mheParams.N_mhe+1;
        SX theta = SX::sym("theta"); SX x = SX::sym("x"); SX y = SX::sym("y"); SX beta0 = SX::sym("beta0");  
        SX u1 = SX::sym("u1"); SX u2 = SX::sym("u2"); 

        SX states = vertcat(theta, x, y, beta0); 
        SX controls = vertcat(u1, u2);  //u1 =dbeta     u2= linear vel 
        unsigned int n_states = states.size1(); unsigned int n_controls = controls.size1();

        SX rhs = vertcat(u2*(1/params.L)*tan(beta0), u2*cos(theta), u2*sin(theta), u1); //system r.h.s  theta, x, y, beta0
        Function f = Function("f",{states,controls}, {rhs}); //nonlinear mapping function f(x,u)
        
        SX U = SX::sym("U",n_controls,A);     //A = N_MHE 
        SX X = SX::sym("X",n_states,B);       //B = N_MHE +1
        SX P = SX::sym("P",n_states,B+A+B+A); //state,control,covOfState,covOfControl 
        
        SX obj = 0;
        SX g;

        for(int k = 0; k < B; k++) // Create states objective function 
        {
            SX h_x = vertcat(X(0,k), X(1,k), X(2,k), X(3,k));
            SX y_tilde = vertcat(P(0,k), P(1,k), P(2,k), P(3,k));
            SX V = SX::zeros(n_states,n_states);
            V(0,0) = P(0,B+A+k); 
            V(1,1) = P(1,B+A+k); 
            V(2,2) = P(2,B+A+k); 
            V(3,3) = P(3,B+A+k);
            obj = obj + mtimes(mtimes(((y_tilde - h_x).T()), V), (y_tilde - h_x));
        }
        for(int k = 0; k < A; k++) // Create control objective function 
        {
            SX con = vertcat(U(0,k), U(1,k));
            SX u_tilde = vertcat(P(0,B+k), P(1,B+k));
            SX W = SX::zeros(n_controls,n_controls);
            W(0,0) = P(0,B+A+B+k); 
            W(1,1) = P(1,B+A+B+k); 
            obj = obj + mtimes(mtimes(((u_tilde - con).T()), W), (u_tilde - con));
        }

        for(int k = 0; k < A; k++) //  multiple shooting constraints
        {
            SX con = vertcat(U(0,k), U(1,k));
            SX st = vertcat(X(0,k), X(1,k), X(2,k), X(3,k));
            SX st_next = vertcat(X(0,k+1), X(1,k+1), X(2,k+1), X(3,k+1));
            SXVector func = f(SXVector{st,con});
            SX f_value = SX::vertcat(func);
            SX st_next_euler = st + ((1/mheParams.loopRate)*f_value);
            g = vertcat(g, (st_next-st_next_euler));
        }

        SX OPT_variables; //optimization variable

        for(int j = 0; j < X.size2(); j++) //reshape X and apend
        {
            for(int i = 0; i < X.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, X(i,j));
            }
        }
        for(int j = 0; j < U.size2(); j++) //reshape U and apend
        {
            for(int i = 0; i < U.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, U(i,j));
            }
        }

        
        SXDict nlp = {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}};
        Dict opts;
        opts["ipopt.max_iter"] = 2000;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = false;
        opts["ipopt.acceptable_tol"] = 1e-6;
        opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
        //opts["ipopt.print_timing_statistics"] = "no";
        //opts["ipopt.print_info_string"] = "no";
        solver = nlpsol("nlpsol", "ipopt", nlp, opts);
        
    }

    void mheSetupTrailer(casadi::Function& solver,const mhe_estimator::CarParams& params,const mhe_estimator::MheParams& mheParams)
    {
        using namespace casadi;
        double A = mheParams.N_mhe;
        double B = mheParams.N_mhe+1;
        SX beta1 = SX::sym("beta1"); SX theta = SX::sym("theta"); SX x = SX::sym("x"); SX y = SX::sym("y"); SX beta0 = SX::sym("beta0"); 
        SX u1 = SX::sym("u1"); SX u2 = SX::sym("u2"); 

        SX states1 = vertcat(beta1,theta, x); 
        SX states2 = vertcat(y, beta0); 
        SX states = vertcat(states1, states2); // due to casadi limitation
        SX controls = vertcat(u1, u2);  //u1 =dbeta     u2= linear vel 
        unsigned int n_states = states.size1(); unsigned int n_controls = controls.size1();

        SX rhs;
        SX k1 = (1/params.L1)*tan(beta1 - atan((params.Lh1/params.L)*tan(beta0)));
        if(!params.moveGuidancePoint)  //OneTrailerKinematicsGPRear
        {
            SX dq0 = u2 * (sin(beta1)/params.Lh1 - (1 + (params.L1/params.Lh1)*cos(beta1))*(k1)); //beta1
            SX dq1 = u2 * k1; //theta
            SX dq2 = u2 * cos(theta); //x
            SX dq3 = u2 * sin(theta); //y
            SX dq4 = u1; //beta0
            
            SX rhs1 = vertcat(dq0, dq1, dq2); //due to casadi limitation 
            SX rhs2 = vertcat(dq3, dq4);
            rhs = vertcat(rhs1, rhs2); //system r.h.s
        } 
        else    //OneTrailerKinematicsGPFront
        {
            SX dq0 = u2 * (sin(beta1)/params.Lh1 - (1 + (params.L1/params.Lh1)*cos(beta1))*k1); //beta1
            SX dq1 = u2 * ( -(params.L1/params.Lh1)*cos(beta1)*k1 + sin(beta1)/params.Lh1 ); //theta
            SX dq2 = u2 * cos(theta) * ( params.L1*sin(beta1)*k1 + cos(beta1) ); //x
            SX dq3 = u2 * sin(theta) * ( params.L1*sin(beta1)*k1 + cos(beta1) ); //y
            SX dq4 = u1; //beta0
            
            
            SX rhs1 = vertcat(dq0, dq1, dq2); //due to casadi limitation 
            SX rhs2 = vertcat(dq3, dq4);
            rhs = vertcat(rhs1, rhs2); //system r.h.s
        }
            
        //ROS_INFO_STREAM("rhstRAILER: " << rhs <<" ");
        
        Function f = Function("f",{states,controls}, {rhs}); //nonlinear mapping function f(x,u)
        
        SX U = SX::sym("U",n_controls,A); //A = N_MHE 
        SX X = SX::sym("X",n_states,B); //B = N_MHE +1
        SX P = SX::sym("P",n_states,B+A+B+A); 
            
        SX obj = 0;
        SX g;
        
        for(int k = 0; k < B; k++) // Create states objective function 
        {
            SX h_x1 = vertcat(X(0,k), X(1,k), X(2,k));
            SX h_x2 = vertcat(X(3,k), X(4,k) );
            SX h_x = vertcat(h_x1, h_x2);

            SX y_tilde1 = vertcat(P(0,k), P(1,k), P(2,k));
            SX y_tilde2 = vertcat(P(3,k), P(4,k));
            SX y_tilde = vertcat(y_tilde1, y_tilde2);

            SX V = SX::zeros(n_states,n_states);
            V(0,0) = P(0,B+A+k); 
            V(1,1) = P(1,B+A+k); 
            V(2,2) = P(2,B+A+k); 
            V(3,3) = P(3,B+A+k);
            V(4,4) = P(4,B+A+k);
            obj = obj + mtimes(mtimes(((y_tilde - h_x).T()), V), (y_tilde - h_x));
        }
        for(int k = 0; k < A; k++) // Create control objective function 
        {
            SX con = vertcat(U(0,k), U(1,k));
            SX u_tilde = vertcat(P(0,B+k), P(1,B+k));
            SX W = SX::zeros(n_controls,n_controls);
            W(0,0) = P(0,B+A+B+k); 
            W(1,1) = P(1,B+A+B+k); 
            obj = obj + mtimes(mtimes(((u_tilde - con).T()), W), (u_tilde - con));
        }

        for(int k = 0; k < A; k++) //  multiple shooting constraints
        {
            SX con = vertcat(U(0,k), U(1,k));

            SX st1 = vertcat(X(0,k), X(1,k), X(2,k));
            SX st2 = vertcat(X(3,k), X(4,k));
            SX st = vertcat(st1,st2);


            SX st_next1 = vertcat(X(0,k+1), X(1,k+1), X(2,k+1));
            SX st_next2 = vertcat(X(3,k+1), X(4,k+1));
            SX st_next = vertcat(st_next1, st_next2);

            SXVector func = f(SXVector{st,con});
            SX f_value = SX::vertcat(func);
            SX st_next_euler = st + ((1/mheParams.loopRate)*f_value);
            g = vertcat(g, (st_next-st_next_euler));
        }

        //ROS_INFO_STREAM("g " << g <<" ");
        
        SX OPT_variables; //optimization variable
        
        for(int j = 0; j < X.size2(); j++) //reshape X and apend
        {
            for(int i = 0; i < X.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, X(i,j));
            }
        }
        for(int j = 0; j < U.size2(); j++) //reshape U and apend
        {
            for(int i = 0; i < U.size1(); i++)
            {
                OPT_variables = vertcat(OPT_variables, U(i,j));
            }
        }
        //ROS_INFO_STREAM("g.size " << g.size1() <<" ");

        
        SXDict nlp = {{"f", obj}, {"x", OPT_variables}, {"g", g}, {"p", P}};
        Dict opts;
        opts["ipopt.max_iter"] = 2000;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = false;
        opts["ipopt.acceptable_tol"] = 1e-8;
        opts["ipopt.acceptable_obj_change_tol"] = 1e-6;
        //opts["ipopt.print_timing_statistics"] = "no";
        //opts["ipopt.print_info_string"] = "no";
        solver = nlpsol("nlpsol", "ipopt", nlp, opts);
    }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <class T, class T2, long unsigned int A, long unsigned int B>
  void estimateMhe(casadi::DM& argx0, const boost::array<T, B>& q4wLoc,
      const boost::array<T2, A>& control2w,const boost::array<T, B>& q4wCov,
      const boost::array<T2, A>& control2wCov, const mhe_estimator::CarParams& carParams,
      const mhe_estimator::MheParams& mheParams,const casadi::Function& solver)  
  {
    using namespace casadi;  
    unsigned int n_states = 4; unsigned int n_controls = 2;
    std::map<std::string, DM> arg, res;
    arg["lbg"] = SX::zeros(n_states*A); //size = n_state * N_Mhe 
    arg["ubg"] = SX::zeros(n_states*A);
    
    std::vector<double> lbx((n_controls*A)+(n_states*B), 0.0);
    std::vector<double> ubx((n_controls*A)+(n_states*B), 0.0);
    
    for(int i = 0; i < (n_states*B); i += n_states) 
    {
        lbx[i] = carParams.lowerTh;  //theta lower bound
        lbx[i+1]   = carParams.lowerX;    //x lower bound
        lbx[i+2] = carParams.lowerY;  //y lower bound
        lbx[i+3] = -carParams.steeringLimit;
    
      
        ubx[i] = carParams.upperTh;   //theta upper bound
        ubx[i+1]   = carParams.upperX;    //x upper bound
        ubx[i+2] = carParams.upperY;  //y upper bound
        ubx[i+3] = carParams.steeringLimit; //beta upper bound
      
    }
    for(int i = ((n_states*B)); i < ((n_controls*A)+(n_states*B)); i += n_controls) 
    {
        lbx[i] = -carParams.SteeringVelLimit;    //v lower bound
        lbx[i+1] = -carParams.LinearVelLimit;  //beta lower bound
    

        ubx[i] = carParams.SteeringVelLimit;    //v upper bound
        ubx[i+1] = carParams.LinearVelLimit;  //beta upper bound        
        
    }
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    //--------------ALL OF THE ABOVE IS JUST A PROBLEM SET UP-------------//
    
    
    DM argp = SX::zeros(n_states,B+A+B+A);
    for(int j = 0; j < B; j++) //measured states
    {
        boost::array<double, 4> qloc = q4wLoc[j];
        argp(0,j) = qloc[0];
        argp(1,j) = qloc[1];
        argp(2,j) = qloc[2];
        argp(3,j) = qloc[3];
    }
    for(int j = 0; j < A; j++) //measured controls
    {
        boost::array<double, 2> qctr = control2w[j];
        //argp(0,j + B) = 0.0;   //we do not have measured control dbeta 
        argp(1,B+j) = qctr[1];
    }
    for(int j = 0; j < B; j++) //state cov matrix
    {
        boost::array<double, 4> qCov = q4wCov[j];
        argp(0,B+A+j) = qCov[0];
        argp(1,B+A+j) = qCov[1];
        argp(2,B+A+j) = qCov[2];
        argp(3,B+A+j) = qCov[3];
    }
    for(int j = 0; j < A; j++) //control cov matrix
    {
        boost::array<double, 2> ctrCov = control2wCov[j];
        //argp(0,B+A+B+j) = 0.0;   //we do not have measured control dbeta 
        argp(1,B+A+B+j) = ctrCov[1];
    }

    arg["p"] = argp;
    arg["x0"] = argx0; //Comes as function parameter

    res = solver(arg); // solve MHE
    argx0 = res.at("x");
    
  }
  


  class MheReset
  {
    ros::ServiceServer srv_;
    bool resetCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        //rest mhe here,
        res.success = true;
        std::string str("MHE window reset successfully.");
        ROS_INFO_STREAM("MHE window reset successfully");
        res.message = str;
        
        return true;
    }
    public:
        MheReset (ros::NodeHandle& nh)
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
    const unsigned long int N_mhe = 200;
    boost::array<Vec4, 201> q4wLoc;        //x y theta beta window
    boost::array<Vec2, 200> control2w;           //control2 window

    boost::array<Vec4, 201> q4wCov;        //state cov window
    boost::array<Vec2, 200> control2wCov;        //control cov window

    boost::array<Vec5, 201> q5wLocTrailer; //x y theta beta window triler =1 
    boost::array<Vec2, 200> control2wTrailer;    //control2 window triler =1 

    boost::array<Vec5, 201> q5wCovTrailer; //state cov window trailer1
    boost::array<Vec2, 200> control2wCovTrailer; //control cov window trailer1 

    Vec4 qLocMhe;               //triler == 0 
    Vec2 controls;              //triler == 0 
    Vec4 qEstFirstSample;       //triler == 0 

    Vec4 qCovMhe;               //cov triler == 0 
    Vec2 controlsCov;           //cov triler == 0 

    Vec5 qLocMheTrailer;         //triler == 1 
    Vec2 controlsTrailer;        //triler == 1 
    Vec5 qEstFirstSampleTrailer; //triler == 1 

    Vec5 qCovMheTrailer;         //Cov triler == 1 
    Vec2 controlsCovTrailer;     //cov triler == 1 

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

    while (::ros::ok())
    {
        MheReset MheReset(nh);
        
        *poseWithCovarianceData = poseWithCovarianceIn();
        ::ros::Duration timeDiff = poseWithCovarianceData->header.stamp - lastPerceptionTime ;
        lastPerceptionTime = poseWithCovarianceData->header.stamp;                           
        bool isPerceptionPoseFresh = timeDiff.toSec() >= 0.01;

        *articulatedAnglesData = articulatedAnglesIn();
        *ackermannDriveData = ackermannDriveIn();
     


        if (carParams.TrailerNumber == 0)
        {

            qLocMhe[0] = poseWithCovarianceData->pose.pose.orientation.z;
            qLocMhe[1] = poseWithCovarianceData->pose.pose.position.x;
            qLocMhe[2] = poseWithCovarianceData->pose.pose.position.y;
            qLocMhe[3] = ackermannDriveData->steering_angle;

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
                q4wLoc[N_mhe - sampleNumber] = qLocMhe;
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

            poseWithCovarianceMheData->pose.pose.orientation.z = qCarEstMhe[0];
            poseWithCovarianceMheData->pose.pose.position.x = qCarEstMhe[1];
            poseWithCovarianceMheData->pose.pose.position.y = qCarEstMhe[2];
            poseWithCovarianceMheOut(poseWithCovarianceMheData);

            ackermannDriveMheData->steering_angle = qCarEstMhe[3];
            ackermannDriveMheData->speed = ctrlEstMhe[1];
            ackermannDriveMheData->steering_angle_velocity = ctrlEstMhe[0];
            ackermannDriveMheOut(ackermannDriveMheData);

        }else if (carParams.TrailerNumber == 1)
        {




            //estimateMheTrailer(argx0Trailer,qMheTrailer, ctrMheTrailer, q5wLocTrailer, control2wTrailer, q5wCovTrailer, control2wCovTrailer, carParams, qFinal, mheParams,solverTrailer);
        }else
        {
            ROS_ERROR(" Trailer Number out of range, accepted values: 1, 2");
        }
        
        ROS_INFO_STREAM("I'm spinning");
        ::ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
