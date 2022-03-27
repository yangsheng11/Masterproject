#include "Tools.h"

namespace MMM
{
float euclidean_distance_help(const Eigen::VectorXd &jointangles, const Eigen::VectorXd &jointangles1){
        float a,sum=0;
        for(auto i=0;i<jointangles.size();i++){
            sum+=pow(jointangles(i)-jointangles1(i),2);
        }
        a=pow(sum,0.5);
        return a;
}
float HDPI_calculator(TrajectoryResult result1,TrajectoryResult result2){
    float l,max=0,c,hdpi;
    size_t a=result1.joint_angles.rows(),b=result1.joint_angles.cols();
    Eigen::VectorXd initial,final,help1,help2;
    initial.resize(b),final.resize(b),help1.resize(b),help2.resize(b);
    initial=result2.joint_angles.row(0),final=result2.joint_angles.row(a-1);
    l=euclidean_distance_help(initial,final);
    for(auto i=0;i<result1.time.size();i++){
         help1=result1.joint_angles.row(i);
         float min=10;
         for(auto j=0;j<result2.time.size();j++){
             help2=result2.joint_angles.row(j);
             c=euclidean_distance_help(help1,help2);
             if(c<min){
                 min=c;
             }
         }
         if(min>max){
             max=min;
         }
    }
    hdpi=max/l;
    return hdpi;
}
float HDPI_calculator_quatenion(TrajectoryResult result1,TrajectoryResult result2){
    float l,max=0,c,hdpi;
    size_t a=result1.joint_angles.rows(),b=result1.joint_angles.cols();
    Eigen::VectorXd initial,final,help1,help2;
    initial.resize(b),final.resize(b),help1.resize(b),help2.resize(b);
    initial=result2.joint_angles.row(0),final=result2.joint_angles.row(a-1);
    l=quatenion_distance_help(initial,final);
    for(auto i=0;i<result1.time.size();i++){
         help1=result1.joint_angles.row(i);
         float min=10;
         for(auto j=0;j<result2.time.size();j++){
             help2=result2.joint_angles.row(j);
             c=quatenion_distance_help(help1,help2);
             if(c<min){
                 min=c;
             }
         }
         if(min>max){
             max=min;
         }
    }
    hdpi=max/l;
    return hdpi;
}
Eigen::VectorXd FPDI_calculator(TrajectoryResult result1,TrajectoryResult result2,const Eigen::VectorXd& max){
    Eigen::VectorXd fpdi,difference;
    difference.resize(max.size()),fpdi.resize(max.size());
    for(auto j=0;j<max.size();j++){
        double ma=0,c;
         for(auto i=0;i<result1.time.size();i++){
             c=abs(result1.joint_angles(i,j)-result2.joint_angles(i,j));
             if(c>ma){
                 ma=c;
             }
         }
         difference[j]=ma;
    }
    for(auto i=0;i<max.size();i++){
        fpdi[i]=difference[i]/max[i];
    }
    return fpdi;
}
float quatenion_distance_help(const Eigen::VectorXd &jointangles, const Eigen::VectorXd &jointangles1){
    float a;
    a=jointangles.dot(jointangles1);
    return acos(a);
}
TrajectoryResult get_TCP_trajectory(TrajectoryResult result,VirtualRobot::RobotNodeSetPtr rns){
    TrajectoryResult tcp;
    tcp.time.resize(result.time.size());
    tcp.joint_angles.resize(result.time.size(),3);
    tcp.joint_velocities.setZero();
    Eigen::VectorXd interangles,tcp1;
    for(auto i=0;i<tcp.time.size();i++){
         tcp.time[i]=result.time[i];
         interangles=result.joint_angles.row(i);
         rns->setJointValues(interangles.cast<float>());
         tcp1 = rns->getTCP()->getGlobalPosition().cast<double>();
         tcp.joint_angles.row(i)=tcp1;
    }
    return tcp;
}
TrajectoryResult get_qua_trajectory(TrajectoryResult result,VirtualRobot::RobotNodeSetPtr rns){
    TrajectoryResult qua;
    qua.time.resize(result.time.size());
    qua.joint_angles.resize(result.time.size(),4);
    qua.joint_velocities.setZero();
    Eigen::VectorXd interangles;
    Eigen::Matrix3f interorientation;
    Eigen::Quaternionf q;
    Eigen::Vector4d interqua;
    for(auto i=0;i<qua.time.size();i++){
        qua.time[i]=result.time[i];
        interangles=result.joint_angles.row(i);
        rns->setJointValues(interangles.cast<float>());
        interorientation=rns->getTCP()->getGlobalOrientation();
        q=simox::math::mat3f_to_quat(interorientation);
        interqua[0]=(double)q.w(),interqua[1]=(double)q.x(),interqua[2]=(double)q.y(),interqua[3]=(double)q.z();
        qua.joint_angles.row(i)=interqua;
    }
    return qua;
}
}
