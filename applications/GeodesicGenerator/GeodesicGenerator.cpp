/*

*/
#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <WholeBodyMotion/WholeBodyMotionLib/Tools.h>
#include <VirtualRobot/XML/RobotIO.h>
#include<time.h>
#include <WholeBodyMotion/WholeBodyMotionLib/BVP_Solver.h>
#include <SimoxUtility/math/convert.h>
#include <math.h>
using namespace std;
using namespace boost::numeric::odeint;
using namespace MMM;

int main(int argc, char *argv[]) {
    float link_mass = 1;
    float link_length = 1;
    float link_inertia=1;

    GeodesicMotionTools::geodesic_ode_2dof_model geodesic_ode_2dof_model(link_length, link_mass,link_inertia);

    Eigen::Vector2d joint_angles;
    joint_angles << 1.5708, -0.5236;
    //joint_angles << 2,1;
    Eigen::Vector2d joint_velocities;
    joint_velocities << 0.7071, 0.7071;

    const std::string dof2_robotString =
        "<Robot Type='2DoFRobot' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Physics>"
        "   <Mass value='" + std::to_string(link_mass) + "' units='kg'/>"
        "   <CoM location='joint' x='" + std::to_string(link_length * 500) + "' y='0' z='0' units='mm'/>"
        "   <InertiaMatrix>"
        "     <row1 c1='1' c2='0' c3='0'/>"
        "     <row2 c1='0' c2='1' c3='0'/>"
        "     <row3 c1='0' c2='0' c3='1'/>"
        "   </InertiaMatrix>"
        "  </Physics>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='-180' hi='180'/>"
        "    <MaxVelocity value='36' unitsLength='mm' unitsTime='h'/>"
        "    <MaxAcceleration value='36' unitsTime='min'/>"
        "    <MaxTorque value='0.2' units='meter'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Transform>"
        "    <Translation x='" + std::to_string(link_length * 1000) + "' y='0' z='0' units='mm'/>"
        "  </Transform>"
        "   <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='-180' hi='180'/>"
        "    <MaxAcceleration value='-1'/>"
        "	 <MaxVelocity value='-1'/>"
        "	 <MaxTorque value='-1'/>"
        "   </Joint>"
        "  <Physics>"
        "   <Mass value='" + std::to_string(link_mass) + "' units='kg'/>"
        "   <CoM location='joint' x='" + std::to_string(link_length * 500) + "' y='0' z='0' units='mm'/>"
        "   <InertiaMatrix>"
        "     <row1 c1='1' c2='0' c3='0'/>"
        "     <row2 c1='0' c2='1' c3='0'/>"
        "     <row3 c1='0' c2='0' c3='1'/>"
        "   </InertiaMatrix>"
        "  </Physics>"
        " </RobotNode>"
        " <RobotNodeSet name='2dof'>"
        "  <Node name='Joint1'/>"
        "  <Node name='Joint2'/>"
        " </RobotNodeSet>"
        "</Robot>";
    VirtualRobot::RobotPtr dof2_robot = VirtualRobot::RobotIO::createRobotFromString(dof2_robotString);
    VirtualRobot::DynamicsPtr dof2_dynamics(new VirtualRobot::Dynamics(dof2_robot->getRobotNodeSet("2dof"), dof2_robot->getRobotNodeSet("2dof"), false));

    GeodesicMotionTools::geodesic_ode_dynamic_model geodesic_ode_2dof_dynamic_model(dof2_dynamics);

    
    Eigen::Matrix2d expected_inertia_matrix;
    expected_inertia_matrix << 4.3660, 1.6830, 1.6830, 1.2500;
    std::cout << "Expected inertia     :\n" << expected_inertia_matrix << "\n";
    Eigen::Matrix2d inertia_matrix = geodesic_ode_2dof_model.computeInertia(joint_angles);
    std::cout << "Computed inertia (1) :\n" << inertia_matrix << "\n\n";
    std::cout << "Computed inertia (2) :\n" << geodesic_ode_2dof_dynamic_model.computeInertia(joint_angles) << "\n\n";

    std::cout << "\n" << std::endl;
    clock_t start, finish;
    Eigen::Tensor<double, 3> expected_inertia_matrix_derivative(2, 2, 2);
    expected_inertia_matrix_derivative.setValues({{{0.0, 0.5}, {0.0, 0.25}}, {{0.0, 0.25}, {0.0, 0.1}}});
    Eigen::Tensor<double, 3> inertia_matrix_derivative = geodesic_ode_2dof_model.computeInertiaDerivative(joint_angles);
    std::cout << "Expected inertia derivative           :\n" << expected_inertia_matrix_derivative << "\n";
    std::cout << "Computed inertia derivative (2dof)    :\n" << inertia_matrix_derivative << "\n\n";
    start=clock();
    std::cout << "Computed inertia derivative (1approx) : \n" << geodesic_ode_2dof_dynamic_model.computeInertiaDerivative(joint_angles, GeodesicMotionTools::geodesic_ode_dynamic_model::FirstOrderApprox) << "\n\n";
    finish=clock();
    std::cout <<"the time cost of lapprox is:" << double(finish - start) / CLOCKS_PER_SEC<<std::endl;
    start=clock();
    std::cout << "Computed inertia derivative (Napprox) : \n" << geodesic_ode_2dof_dynamic_model.computeInertiaDerivative(joint_angles, GeodesicMotionTools::geodesic_ode_dynamic_model::NOrderApprox) << "\n\n";
    finish=clock();
    std::cout <<"the time cost of Napprox is:" << double(finish - start) / CLOCKS_PER_SEC<<std::endl;
    start=clock();
    std::cout << "Computed inertia derivative (CRBA)    : \n" << geodesic_ode_2dof_dynamic_model.computeInertiaDerivative(joint_angles, GeodesicMotionTools::geodesic_ode_dynamic_model::CompositeRigidBodyAlgorithmDerivation) << "\n\n";
    finish=clock();
    std::cout <<"the time cost of CRBA is:" << double(finish - start) / CLOCKS_PER_SEC<<std::endl;

    Eigen::Vector2d expected_acceleration;
    expected_acceleration << -0.2587, 0.4483;
    Eigen::VectorXd acceleration = geodesic_ode_2dof_model.compute_geodesic_acceleration(joint_velocities, inertia_matrix, inertia_matrix_derivative);
    std::cout << "Expected acceleration:\n" << expected_acceleration << "\n";
    std::cout << "Computed acceleration:\n" << acceleration << "\n\n";

    MMM::TrajectoryResult result = GeodesicMotionTools::solve_geodesic_ivp(joint_angles, joint_velocities, geodesic_ode_2dof_model);
    /*for(auto i=0;i<result.time.size();i++){
    std::cout<<"ode.time"<<result.time[i]<<"\n"<<"ode.jointangles"<<result.joint_angles.row(i)<<"\n"<<"ode.jointvelocity"<<result.joint_velocities.row(i)<<std::endl;
    }
    Eigen::Vector2d final_joint_angles,guess_velocity;
    final_joint_angles<<2.2096,0.2997;
    guess_velocity<<1.1,1.1;
    MMM::TrajectoryResult result1=GeodesicMotionTools::solve_geodesic_bvp(joint_angles,final_joint_angles,guess_velocity,geodesic_ode_2dof_model);
    for(auto i=0;i<result1.time.size();i++){
    std::cout<<"ode.time  1"<<result1.time[i]<<"\n"<<"ode-bvp.jointangles"<<result1.joint_angles.row(i)<<"\n"<<"ode-bvp.jointvelocity"<<result1.joint_velocities.row(i)<<std::endl;
    }*/
    /*double T=1,initial_speed=0,final_speed=0,t=0.5,t1=0.51;
    float s=geodesic_ode_2dof_model.compute_curvelenth_time(T,initial_speed,final_speed,t);
    float s1=geodesic_ode_2dof_model.compute_curvelenth_time(T,initial_speed,final_speed,t1);
    Eigen::VectorXd jointangles_s=geodesic_ode_2dof_model.compute_jointangles_curvelenth(result,s);
    Eigen::VectorXd jointangles_s1=geodesic_ode_2dof_model.compute_jointangles_curvelenth(result,s1);
    std::cout<<"q(s)"<<jointangles_s1<<"dq at 1"<<(jointangles_s1-jointangles_s)/0.01<<std::endl;*/
    VirtualRobot::RobotPtr mmm_robot = VirtualRobot::RobotIO::loadRobot(std::string(PROJECT_DATA_DIR) + "Models/Winter/mmm.xml");
    VirtualRobot::RobotNodeSetPtr rns = mmm_robot->getRobotNodeSet("LeftArm-7dof");
    VirtualRobot::DynamicsPtr mmm_dynamics = VirtualRobot::DynamicsPtr(new VirtualRobot::Dynamics(rns, mmm_robot->getRobotNodeSet("LeftArm-7dof"), false));
    Eigen::Matrix3f orien=rns->getTCP()->getGlobalOrientation();
    Eigen::Quaternionf q=simox::math::mat3f_to_quat(orien);
    double angle=(acos(q.w())*180/M_PI)*2;
    std::cout<<"orien"<<q.w()<<"\n"<<angle<<std::endl;
    /*Eigen::VectorXd bvpinitial,bvpfinal,final_2d0fs;
    bvpinitial.resize(7),bvpfinal.resize(7),final_2d0fs.resize(2);
    bvpinitial<<0.0042,0.00197,0.00392,0.004,0.004558,0.000988,0.001676;
    bvpfinal<<0.435715,0.0679,0.311,0.1249,0.3963,0.2828,0.2294;
    final_2d0fs<<2.20957,0.299724;
    double T=1.5;
    MMM::TrajectoryResult result_bvp=solve_geodesic_bvp_A(bvpinitial,bvpfinal,mmm_dynamics,T);
    for(auto i=0;i<result_bvp.time.size();i++){
        std::cout<<"bvp.time"<<result_bvp.time[i]<<"\n"<<"bvp.jointangles"<<result_bvp.joint_angles.row(i)<<"\n"<<"bvp.jointvelocity"<<result_bvp.joint_velocities.row(i)<<std::endl;
    }*/
    exit(1); // Remove for mmm model testing - Currently not working well as Robot Node Sets have to be adjusted

    // MMM model:

    //std::cout << std::endl << "\n\n\nMMM:\n\n";
    GeodesicMotionTools::geodesic_ode_dynamic_model geodesic_ode_mmm(mmm_dynamics);
    Eigen::VectorXd jointangles_7,jointvelocity_7,jointhelp;
    jointangles_7.resize(7,1),jointvelocity_7.resize(7,1),jointhelp.resize(7,1);
    jointangles_7<<0.00420094,0.00197191,0.0039155,0.0039922,0.00455824,0.000987757,0.00167611;
    jointhelp<<0.00804209,0.00336079,0.00668535,0.00637919,0.00770259,0.00281168,0.00424312;
    jointvelocity_7=(jointhelp-jointangles_7)/0.01;
    MMM::TrajectoryResult result1=GeodesicMotionTools::solve_geodesic_ivp(jointangles_7,jointvelocity_7,geodesic_ode_mmm);
    for(auto i=0;i<result1.time.size();i++){
    std::cout<<"ode7.time"<<result1.time[i]<<"\n"<<"ode7.jointangles"<<result1.joint_angles.row(i)<<std::endl;
    }
    Eigen::VectorXd jointValues = rns->getJointValuesEigen().cast<double>(); // initial joint angles
    /*std::map<std::string,float> map1=rns->getJointValueMap();
    for(auto &v:rns->getNodeNames()) std::cout<<v<<std::endl;
    std::cout<<map1["LSx_joint"]<<std::endl;*/
    /*std::cout << "MMM - Computed inertia :\n" << geodesic_ode_mmm.computeInertia(jointValues) << "\n\n";
    std::cout << "MMM - Computed inertia derivative (1approx) : \n" << geodesic_ode_mmm.computeInertiaDerivative(jointValues, GeodesicMotionTools::geodesic_ode_dynamic_model::FirstOrderApprox) << "\n\n";
    std::cout << "MMM - Computed inertia derivative (Napprox) : \n" << geodesic_ode_mmm.computeInertiaDerivative(jointValues, GeodesicMotionTools::geodesic_ode_dynamic_model::NOrderApprox) << "\n\n";
    std::cout << "MMM - Computed inertia derivative (CRBA)    : \n" << geodesic_ode_mmm.computeInertiaDerivative(jointValues, GeodesicMotionTools::geodesic_ode_dynamic_model::CompositeRigidBodyAlgorithmDerivation) << "\n\n";
    std::cout << std::endl;*/
}

