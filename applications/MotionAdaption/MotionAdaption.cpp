/*
This file is part of MMM.

MMM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MMM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MMM.  If not, see <http://www.gnu.org/licenses/>.
*
* @package    MMM
* @author     Andre Meixner
* @copyright  2020 High Performance Humanoid Technologies (H2T), Karlsruhe, Germany
*
*/

#include "MotionAdaptionConfiguration.h"

#include <string>
#include <vector>
#include <tuple>
#include<fstream>
#include <MMM/Motion/MotionReaderXML.h>
#include <MMM/Model/ModelReaderXML.h>
#include <MMM/Model/ModelProcessorWinter.h>
#include <MMM/Motion/Motion.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Dynamics/Dynamics.h>
#include <WholeBodyMotion/WholeBodyMotionLib/finite_difference.hpp>
// #include<boost/math/differentiation/finite_difference.hpp>
#include <MMM/Motion/Plugin/ModelPosePlugin/ModelPoseSensor.h>
#include <MMM/Motion/Plugin/ModelPosePlugin/ModelPoseSensorMeasurement.h>
#include <MMM/Motion/Plugin/KinematicPlugin/KinematicSensor.h>
#include <MMM/Motion/Plugin/KinematicPlugin/KinematicSensorMeasurement.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <WholeBodyMotion/WholeBodyMotionLib/Tools.h>
#include <WholeBodyMotion/WholeBodyMotionLib/BVP_Solver.h>
#include<iomanip>
#include <SimoxUtility/math/convert.h>
using namespace  boost::math::differentiation;
using namespace MMM;
using namespace std;
int main(int argc, char *argv[]) {
    MMM_INFO << "--- Whole Body Motion Analysis ---" << std::endl;

    MotionAdaptionConfiguration *configuration = new MotionAdaptionConfiguration();
    if (!configuration->processCommandLine(argc,argv)) {
        MMM_ERROR << "Could not process command line, aborting." << std::endl;
        return -1;
    }

    configuration->print();

   try {
        MMM::MotionReaderXMLPtr motionReader(new MMM::MotionReaderXML(true, false));
        MMM_INFO << "Reading motion file '" << configuration->inputMotionPath << "'!" << std::endl;
        MMM::MotionRecordingPtr motions = motionReader->loadMotionRecording(configuration->inputMotionPath); // can also load from directory
        MMM::MotionPtr motion = motions->getReferenceModelMotion();
        if (!motion) {
            MMM_ERROR << "No mmm motion!" << std::endl;
            return -2;
        }

        VirtualRobot::RobotPtr model = motion->getModel();
        VirtualRobot::RobotNodeSetPtr rns = model->getRobotNodeSet("LeftArm-7dof");

        MMM::KinematicSensorPtr kinematicSensor(new MMM::KinematicSensor(rns->getNodeNames()));
        VirtualRobot::DynamicsPtr mmm_dynamics(new VirtualRobot::Dynamics(rns, motion->getModel()->getRobotNodeSet("All"), false));

        GeodesicMotionTools::geodesic_ode_dynamic_model geodesic_ode_mmm(mmm_dynamics);

        bool tmp = true;
        Eigen::VectorXd max;
        max.resize(rns->getSize());
        max[0]=3.31613+1.22713,max[1]=1.0472+1.22173,max[2]=2.79253,max[3]=2.79253,max[4]=1.5708*2,max[5]=0.349066+0.523599,max[6]=1.5708+1.22173;


        if (!tmp) {


            std::vector<float> splitTimesteps = { 1.2f, 1.7f, 2.2f };

            splitTimesteps.insert(splitTimesteps.begin(), motion->getMinTimestep());
            splitTimesteps.push_back(motion->getMaxTimestep());

                float startTimestep = splitTimesteps[0];
                motion->initializeModel(model, startTimestep);
                Eigen::VectorXd initialConfiguration = rns->getJointValuesEigen().cast<double>();

                float endTimestep = splitTimesteps[1];
                motion->initializeModel(model, endTimestep);
                Eigen::VectorXd finalConfiguration = rns->getJointValuesEigen().cast<double>();
                double T=endTimestep-startTimestep;
                //MMM::TrajectoryResult result = solve_geodesic_bvp_A(initialConfiguration, finalConfiguration, mmm_dynamics, T);
                MMM::TrajectoryResult result = euclidean_motion_solver(initialConfiguration, finalConfiguration,T);


                for (float timestep = startTimestep; timestep < endTimestep; timestep += 0.01f) {
                    // set the values
                    Eigen::VectorXf configuration(rns->getSize());
                    Eigen::VectorXd acceleration,sprayfield,velocity_now,velocity_next,jointangles;
                    velocity_now.resize(rns->getSize()),velocity_next.resize(rns->getSize()),jointangles.resize(rns->getSize());
                    for (unsigned int i = 0; i < configuration.rows(); i++){
                        configuration(i)=result.joint_angles((timestep-0)*100,i);
                        jointangles(i)=result.joint_angles((timestep-0)*100,i);
                        velocity_now(i)=result.joint_velocities((timestep-0)*100,i);
                        velocity_next(i)=result.joint_velocities((timestep-0)*100+1,i);
                    }
                    rns->setJointValues(configuration);

                    acceleration=(velocity_next-velocity_now)/0.01;
                    Eigen::MatrixXd inertia;
                    Eigen::Tensor<double,3> inertiaderi;
                    geodesic_ode_mmm.computeInertiaAndDerivative(jointangles, inertia, inertiaderi);
                    sprayfield=geodesic_ode_mmm.compute_geodesic_acceleration(velocity_now,inertia,inertiaderi);

                    float sum=0;
                    for(unsigned int i=0; i<rns->getSize(); i++){
                        sum+=sprayfield(i)-acceleration(i);
                    }

                    std::cout << timestep << ": " << "ERROR"<<sum << std::endl;

                    MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration));
                    kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
                }

              for (unsigned int i = 2; i < splitTimesteps.size(); i++) {
                float startTimestep1 = splitTimesteps[i-1];
                motion->initializeModel(model, startTimestep1);
                Eigen::VectorXd initialConfiguration1 = rns->getJointValuesEigen().cast<double>();
                float endTimestep1 = splitTimesteps[i];
                motion->initializeModel(model, endTimestep1);
                Eigen::VectorXd finalConfiguration1 = rns->getJointValuesEigen().cast<double>();
                double T1=endTimestep1-startTimestep1;
                Eigen::Vector3f position;
                Eigen::Matrix3f orientation;
                //MMM::TrajectoryResult result1 = solve_geodesic_bvp_A(initialConfiguration1, finalConfiguration1, mmm_dynamics, T1);
                MMM::TrajectoryResult result1 = euclidean_motion_solver(initialConfiguration1, finalConfiguration1,T1);
                for (float timestep = startTimestep1; timestep < endTimestep1; timestep += 0.01f) {
                    Eigen::VectorXf configuration1(rns->getSize());
                    for (unsigned int i = 0; i < configuration1.rows(); i++){
                        configuration1(i)=result1.joint_angles((timestep-startTimestep1)*100,i);
                    }
                    std::cout<<"timestep:"<<timestep<<"\n"<<configuration1<<"\n"<<std::endl;
                    rns->setJointValues(configuration1);
                    position=model->getRobotNodeSet("LeftArm-7dof")->getTCP()->getGlobalPosition();
                    orientation=model->getRobotNodeSet("LeftArm-7dof")->getTCP()->getGlobalOrientation();
                    std::cout<<"position:"<<position<<"\n"<<"orientation:"<<orientation<<"\n"<<std::endl;
                    MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration1));
                    kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
                }
              }
        }
        else {
            std::vector<float> splitTimesteps = { 1.2f, 1.7f, 2.2f };

            splitTimesteps.insert(splitTimesteps.begin(), motion->getMinTimestep());
            splitTimesteps.push_back(motion->getMaxTimestep());
                for (unsigned int i = 1; i < splitTimesteps.size(); i++) {
                  float startTimestep1 = splitTimesteps[i-1];
                  motion->initializeModel(model, startTimestep1);
                  Eigen::VectorXd initialConfiguration1 = rns->getJointValuesEigen().cast<double>();
                  Eigen::VectorXd interangles;
                  float endTimestep1 = splitTimesteps[i];
                  motion->initializeModel(model, endTimestep1);
                  Eigen::VectorXd finalConfiguration1 = rns->getJointValuesEigen().cast<double>();
                  double T1=endTimestep1-startTimestep1;
                  double initialv=0,finalv=0;
                  MMM::TrajectoryResult result_fp_eu=euclidean_motion_solver(initialConfiguration1,finalConfiguration1,T1);
                  //MMM::TrajectoryResult result_fp_ri= solve_geodesic_bvp_A(initialConfiguration1, finalConfiguration1, mmm_dynamics, T1);
                  MMM::TrajectoryResult result_eu = get_TCP_trajectory(result_fp_eu,rns),result_eu_qu=get_qua_trajectory(result_fp_eu,rns);
                  //MMM::TrajectoryResult result_ri = get_TCP_trajectory(result_fp_ri,rns),result_ri_qu=get_qua_trajectory(result_fp_ri,rns);
                  MMM::TrajectoryResult result_or_fp;
                  result_or_fp.time.resize(T1/0.01+1);
                  for(auto i=0;i<result_or_fp.time.size();i++){
                      result_or_fp.time[i]=(double)i/100;
                  }
                  result_or_fp.joint_angles.resize(T1/0.01+1,initialConfiguration1.size()),result_or_fp.joint_velocities.resize(T1/0.01+1,initialConfiguration1.size());
                  result_or_fp.joint_velocities.setZero();
                  for(auto i=0;i<result_or_fp.time.size();i++){
                      motion->initializeModel(model, startTimestep1+result_or_fp.time[i]);
                      interangles=rns->getJointValuesEigen().cast<double>();
                      for(auto z=0;z<interangles.size();z++){
                          result_or_fp.joint_angles(i,z)=interangles[z];
                      }
                  }
                  MMM::TrajectoryResult result_or=get_TCP_trajectory(result_or_fp,rns),result_or_qu=get_qua_trajectory(result_or_fp,rns);
                  //float hdpi=HDPI_calculator(result_eu,result_or),hpdi_qua=HDPI_calculator_quatenion(result_eu_qu,result_or_qu);
                  float hdpi=HDPI_calculator(result_eu,result_or),hpdi_qua=HDPI_calculator_quatenion(result_eu_qu,result_or_qu);
                  //Eigen::VectorXd fpdi=FPDI_calculator(result_fp_eu,result_or_fp,max);
                  Eigen::VectorXd fpdi=FPDI_calculator(result_fp_eu,result_or_fp,max);
                  std::cout<<"TCP_hpdi:"<<hdpi<<"\n"<<"Jointangles_FPDI:"<<fpdi<<"\n"<<"Quatenion_HPDI:"<<hpdi_qua<<std::endl;

                  for (float timestep = startTimestep1; timestep < endTimestep1; timestep += 0.01f) {
                      Eigen::VectorXf configuration1(rns->getSize());
                      double t=timestep-startTimestep1;
                      float s=geodesic_ode_mmm.compute_curvelenth_time(T1,initialv,finalv,t);
                      Eigen::VectorXd currentangles=geodesic_ode_mmm.compute_jointangles_curvelenth(result_fp_eu,s);
                      for (unsigned int i = 0; i < configuration1.rows(); i++){
                          configuration1(i)=currentangles(i);
                      }
                      rns->setJointValues(configuration1);
                      MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration1));
                      kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
                  }
                }

        }

        for (MMM::KinematicSensorPtr kinematicSensor : motion->getSensorsByType<MMM::KinematicSensor>()) {
            for (const std::string &jointName : rns->getNodeNames())
                kinematicSensor->removeJoint(jointName);
        }
        motion->addSensor(kinematicSensor);

        motions->saveXML(configuration->outputMotionPath);

        MMM_INFO << "--- END ---" << std::endl;
        return 0;
    }
    catch (MMM::Exception::MMMException &e) {
        MMM_ERROR << e.what() << std::endl;
        return -2;
    }
}
