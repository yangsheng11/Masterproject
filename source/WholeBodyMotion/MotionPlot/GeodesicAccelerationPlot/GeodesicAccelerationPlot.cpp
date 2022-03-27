#include "GeodesicAccelerationPlot.h"
#include <WholeBodyMotion/WholeBodyMotionLib/Tools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <SimoxUtility/math/convert.h>
using namespace MMM;

GeodesicAccelerationPlot::GeodesicAccelerationPlot(BasicKinematicSensorPtr sensor, MotionPtr motion) :
    sensor(sensor),
    motion(motion),
    model(motion->getModel()->clone()),
    rns(model->getRobotNodeSet(RobotNodeSetName))
{
}

std::vector<std::string> GeodesicAccelerationPlot::getNames() {
    return rns->getNodeNames();
}

std::tuple<QVector<double>, QVector<double> > GeodesicAccelerationPlot::getPlot(std::string name) {
    QVector<double> x(sensor->getTimesteps().size()), y(sensor->getTimesteps().size());
    VirtualRobot::DynamicsPtr dynamics_model(new VirtualRobot::Dynamics(rns, model->getRobotNodeSet("All"), false));
    GeodesicMotionTools::geodesic_ode_dynamic_model geodesic_ode_mmm(dynamics_model);
    /*double T=1,initial_speed=0,final_speed=0;*/
    for (size_t i=0;i<sensor->getTimesteps().size();i++) {
        motion->initializeModel(model, sensor->getTimesteps()[i]);
        x[i] = sensor->getTimesteps()[i];
        //int index=rns->getRobotNodeIndex(name);
        Eigen::VectorXd jointanglesrns=rns->getJointValuesEigen().cast<double>();
        Eigen::Matrix3f orientation=model->getRobotNodeSet("RightLeg-7dof")->getTCP()->getGlobalOrientation();
        Eigen::Quaternionf q=simox::math::mat3f_to_quat(orientation);
        //double angle=(acos(q.w())*180/M_PI)*2;
        y[i]=(double)q.w();
        //y[i]=model->getRobotNodeSet("RightLeg-7dof")->getTCP()->getGlobalPosition().cast<double>()(1);
        //y[i]=jointanglesrns[index];
        // currently plotting the joint value as example. Set here the geodesic value:
        //float s=geodesic_ode_mmm.compute_curvelenth_time(T,initial_speed,final_speed,sensor->getTimesteps()[i]);
        //float s1=geodesic_ode_mmm.compute_curvelenth_time(T,initial_speed,final_speed,sensor->getTimesteps()[i+1]);
        //y[i]=(s1-s)/(sensor->getTimesteps()[i+1]-sensor->getTimesteps()[i]);
        /*Eigen::VectorXd jointanglesrns=rns->getJointValuesEigen().cast<double>(),jointvelocity,jointanglesnext;
        Eigen::MatrixXd inertia=dynamics_model->getInertiaMatrix(jointanglesrns,false);
        motion->initializeModel(model,sensor->getTimesteps()[i+1]);
        jointanglesnext=rns->getJointValuesEigen().cast<double>();
        jointvelocity=(jointanglesnext-jointanglesrns)/(sensor->getTimesteps()[i+1]-sensor->getTimesteps()[i]);
        double metricspeed_rns=jointvelocity.transpose()*inertia*jointvelocity;
        y[i]=metricspeed_rns;*/
    }

    return std::make_tuple(x, y);
}

std::string GeodesicAccelerationPlot::getName() {
    return RobotNodeSetName;
}
