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

#include "MotionGenerationConfiguration.h"

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
using namespace  boost::math::differentiation;
using namespace MMM;
using namespace std;
int main(int argc, char *argv[]) {
    MMM_INFO << "--- Whole Body Motion Analysis ---" << std::endl;

    MotionGenerationConfiguration *configuration = new MotionGenerationConfiguration();
    if (!configuration->processCommandLine(argc,argv)) {
        MMM_ERROR << "Could not process command line, aborting." << std::endl;
        return -1;
    }

    configuration->print();

   try {
        MMM::ModelReaderXMLPtr reader(new MMM::ModelReaderXML());
        MMM::ModelProcessorWinterPtr processor(new MMM::ModelProcessorWinter(configuration->height, configuration->weight));
        std::filesystem::path modelPath = std::string(PROJECT_DATA_DIR) + "Models/Winter/mmm.xml";
        MMM::ProcessedModelWrapperPtr model = reader->loadMotionModel(modelPath, configuration->outputMotionPath, processor, false);
        MMM::MotionPtr motion(new MMM::Motion("TestMotion", model, "", MMM::MotionType::MMM));

        MMM::ModelPoseSensorPtr modelPoseSensor(new MMM::ModelPoseSensor());
        Eigen::Matrix4f modelPose = Eigen::Matrix4f::Identity();
        modelPose(2,3) = 1000.0f;
        MMM::ModelPoseSensorMeasurementPtr modelPoseSensorMeasurement(new MMM::ModelPoseSensorMeasurement(0.0, modelPose));
        modelPoseSensor->addSensorMeasurement(modelPoseSensorMeasurement);
        motion->addSensor(modelPoseSensor);
        VirtualRobot::RobotNodeSetPtr rns = model->getModel()->getRobotNodeSet("LeftArm-7dof");
        MMM::KinematicSensorPtr kinematicSensor(new MMM::KinematicSensor(rns->getNodeNames()));
        motion->addSensor(kinematicSensor);
        VirtualRobot::DynamicsPtr mmm_dynamics(new VirtualRobot::Dynamics(rns, model->getModel()->getRobotNodeSet("All"), false));

        GeodesicMotionTools::geodesic_ode_dynamic_model geodesic_ode_mmm(mmm_dynamics);

        bool ivp = true;
        if (!ivp) {
            float startTimestep = 1.0f; // start of your waving motion
            motion->initializeModel(model->getModel(), startTimestep);
            Eigen::VectorXf initialConfiguration = rns->getJointValuesEigen();

            float endTimestep = 2.5f; // end of your waving motion
            motion->initializeModel(model->getModel(), endTimestep);
            Eigen::VectorXf finalConfiguration = rns->getJointValuesEigen();
            double T=1.5;

            Eigen::VectorXd joint_angles,final_angles;
            joint_angles.resize(7,1),final_angles.resize(7,1);
            for (unsigned int i = 0; i < 7; i++){
                joint_angles(i)=initialConfiguration(i);
                final_angles(i)=finalConfiguration(i);
            }
            MMM::TrajectoryResult result = solve_geodesic_bvp_A(joint_angles, final_angles, mmm_dynamics,T);
            //MMM::TrajectoryResult result = euclidean_motion_solver(joint_angles, final_angles,T);
            std::cout<<finalConfiguration<<std::endl;


            for (float timestep = startTimestep; timestep < endTimestep; timestep += 0.01f) {
                // set the values
                Eigen::VectorXf configuration(rns->getSize());
                Eigen::VectorXd acceleration,sprayfield,velocity_now,velocity_next,jointangles;
                velocity_now.resize(7),velocity_next.resize(7),jointangles.resize(7);
                for (unsigned int i = 0; i < configuration.rows(); i++){
                    configuration(i)=result.joint_angles((timestep-1)*100,i);
                    jointangles(i)=result.joint_angles((timestep-1)*100,i);
                    velocity_now(i)=result.joint_velocities((timestep-1)*100,i);
                    velocity_next(i)=result.joint_velocities((timestep-1)*100+1,i);
                }
                rns->setJointValues(configuration);


                acceleration=(velocity_next-velocity_now)/0.01;
                Eigen::MatrixXd inertia;
                Eigen::Tensor<double,3> inertiaderi;
                geodesic_ode_mmm.computeInertiaAndDerivative(jointangles, inertia, inertiaderi);
                sprayfield=geodesic_ode_mmm.compute_geodesic_acceleration(velocity_now,inertia,inertiaderi);

                float sum=0;
                for(auto i=0;i<7;i++){
                    sum+=sprayfield(i)-acceleration(i);
                }

                std::cout << timestep << ": " << "ERROR"<<sum << std::endl;

                MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration));
                kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
            }

            /*float startTimestep2 = 1.2f; // start of your waving motion
            motion->initializeModel(model->getModel(), startTimestep2);
            Eigen::VectorXf initialConfiguration2 = rns->getJointValuesEigen();

            float endTimestep2 = 1.7f; // end of your waving motion
            motion->initializeModel(model->getModel(), endTimestep2);
            Eigen::VectorXf finalConfiguration2 = rns->getJointValuesEigen();
            double T2=0.5;

            Eigen::VectorXd joint_angles2,final_angles2;
            joint_angles2.resize(7,1),final_angles2.resize(7,1);
            for (unsigned int i = 0; i < 7; i++){
                joint_angles2(i)=initialConfiguration2(i);
                final_angles2(i)=finalConfiguration2(i);
            }
            MMM::TrajectoryResult result2 = solve_geodesic_bvp_A(joint_angles2, final_angles2, mmm_dynamics,T2);
            //MMM::TrajectoryResult result2 = euclidean_motion_solver(joint_angles2, final_angles2,T2);


            for (float timestep = startTimestep2; timestep < endTimestep2; timestep += 0.01f) {
                // set the values
                Eigen::VectorXf configuration(rns->getSize());
                Eigen::VectorXd acceleration,sprayfield,velocity_now,velocity_next,jointangles;
                velocity_now.resize(7),velocity_next.resize(7),jointangles.resize(7);
                for (unsigned int i = 0; i < configuration.rows(); i++){
                    configuration(i)=result2.joint_angles((timestep-1)*100,i);
                    jointangles(i)=result2.joint_angles((timestep-1)*100,i);
                    velocity_now(i)=result2.joint_velocities((timestep-1)*100,i);
                    velocity_next(i)=result2.joint_velocities((timestep-1)*100+1,i);
                }
                rns->setJointValues(configuration);

                acceleration=(velocity_next-velocity_now)/0.01;
                Eigen::MatrixXd inertia;
                Eigen::Tensor<double,3> inertiaderi;
                geodesic_ode_mmm.computeInertiaAndDerivative(jointangles, inertia, inertiaderi);
                sprayfield=geodesic_ode_mmm.compute_geodesic_acceleration(velocity_now,inertia,inertiaderi);

                float sum=0;
                for(auto i=0;i<7;i++){
                    sum+=sprayfield(i)-acceleration(i);
                }

                std::cout << timestep << ": " << "ERROR"<<sum << std::endl;

                MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration));
                kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
            }

            float startTimestep3 = 1.7f; // start of your waving motion
            motion->initializeModel(model->getModel(), startTimestep3);
            Eigen::VectorXf initialConfiguration3 = rns->getJointValuesEigen();

            float endTimestep3 = 2.2f; // end of your waving motion
            motion->initializeModel(model->getModel(), endTimestep3);
            Eigen::VectorXf finalConfiguration3 = rns->getJointValuesEigen();
            double T3=0.5;

            Eigen::VectorXd joint_angles3,final_angles3;
            joint_angles2.resize(7,1),final_angles2.resize(7,1);
            for (unsigned int i = 0; i < 7; i++){
                joint_angles3(i)=initialConfiguration3(i);
                final_angles3(i)=finalConfiguration3(i);
            }
            MMM::TrajectoryResult result3 = solve_geodesic_bvp_A(joint_angles3, final_angles3, mmm_dynamics,T3);
            //MMM::TrajectoryResult result3 = euclidean_motion_solver(joint_angles3, final_angles3,T3);


            for (float timestep = startTimestep3; timestep < endTimestep3; timestep += 0.01f) {
                // set the values
                Eigen::VectorXf configuration(rns->getSize());
                Eigen::VectorXd acceleration,sprayfield,velocity_now,velocity_next,jointangles;
                velocity_now.resize(7),velocity_next.resize(7),jointangles.resize(7);
                for (unsigned int i = 0; i < configuration.rows(); i++){
                    configuration(i)=result3.joint_angles((timestep-1)*100,i);
                    jointangles(i)=result3.joint_angles((timestep-1)*100,i);
                    velocity_now(i)=result3.joint_velocities((timestep-1)*100,i);
                    velocity_next(i)=result3.joint_velocities((timestep-1)*100+1,i);
                }
                rns->setJointValues(configuration);

                acceleration=(velocity_next-velocity_now)/0.01;
                Eigen::MatrixXd inertia;
                Eigen::Tensor<double,3> inertiaderi;
                geodesic_ode_mmm.computeInertiaAndDerivative(jointangles, inertia, inertiaderi);
                sprayfield=geodesic_ode_mmm.compute_geodesic_acceleration(velocity_now,inertia,inertiaderi);

                float sum=0;
                for(auto i=0;i<7;i++){
                    sum+=sprayfield(i)-acceleration(i);
                }

                std::cout << timestep << ": " << "ERROR"<<sum << std::endl;

                MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration));
                kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
            }

            float startTimestep4 = 2.2f; // start of your waving motion
            motion->initializeModel(model->getModel(), startTimestep4);
            Eigen::VectorXf initialConfiguration4 = rns->getJointValuesEigen();

            float endTimestep4 = 4.6f; // end of your waving motion
            motion->initializeModel(model->getModel(), endTimestep4);
            Eigen::VectorXf finalConfiguration4 = rns->getJointValuesEigen();
            double T4=2.4;

            Eigen::VectorXd joint_angles4,final_angles4;
            joint_angles2.resize(7,1),final_angles2.resize(7,1);
            for (unsigned int i = 0; i < 7; i++){
                joint_angles4(i)=initialConfiguration4(i);
                final_angles4(i)=finalConfiguration4(i);
            }
            MMM::TrajectoryResult result4 = solve_geodesic_bvp_A(joint_angles4, final_angles4, mmm_dynamics,T4);
            //MMM::TrajectoryResult result4 = euclidean_motion_solver(joint_angles4, final_angles4,T4);


            for (float timestep = startTimestep4; timestep < endTimestep4; timestep += 0.01f) {
                // set the values
                Eigen::VectorXf configuration(rns->getSize());
                Eigen::VectorXd acceleration,sprayfield,velocity_now,velocity_next,jointangles;
                velocity_now.resize(7),velocity_next.resize(7),jointangles.resize(7);
                for (unsigned int i = 0; i < configuration.rows(); i++){
                    configuration(i)=result4.joint_angles((timestep-1)*100,i);
                    jointangles(i)=result4.joint_angles((timestep-1)*100,i);
                    velocity_now(i)=result4.joint_velocities((timestep-1)*100,i);
                    velocity_next(i)=result4.joint_velocities((timestep-1)*100+1,i);
                }
                rns->setJointValues(configuration);

                acceleration=(velocity_next-velocity_now)/0.01;
                Eigen::MatrixXd inertia;
                Eigen::Tensor<double,3> inertiaderi;
                geodesic_ode_mmm.computeInertiaAndDerivative(jointangles, inertia, inertiaderi);
                sprayfield=geodesic_ode_mmm.compute_geodesic_acceleration(velocity_now,inertia,inertiaderi);

                float sum=0;
                for(auto i=0;i<7;i++){
                    sum+=sprayfield(i)-acceleration(i);
                }

                std::cout << timestep << ": " << "ERROR"<<sum << std::endl;

                MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration));
                kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
            }*/

        }
        else {
            Eigen::VectorXd joint_angles,joint_velocities,jointhelp;
            joint_angles.resize(7,1),joint_velocities.resize(7,1),jointhelp.resize(7,1);
            joint_angles<<0.00420094,0.00197191,0.0039155,0.0039922,0.00455824,0.000987757,0.00167611;
            jointhelp<<0.00804209,0.00336079,0.00668535,0.00637919,0.00770259,0.00281168,0.00424312;
            joint_velocities=(jointhelp-joint_angles)/0.01;
            MMM::TrajectoryResult result = GeodesicMotionTools::solve_geodesic_ivp(joint_angles, joint_velocities, geodesic_ode_mmm);
            double T=1,initial_speed=0,final_speed=0;

            for (float timestep = 0.0f; timestep < 1.0f; timestep += 0.01f) {
                float s=geodesic_ode_mmm.compute_curvelenth_time(T,initial_speed,final_speed,timestep);
                Eigen::VectorXd jointangles_s=geodesic_ode_mmm.compute_jointangles_curvelenth(result,s);
                Eigen::VectorXf configuration(rns->getSize());
                for (unsigned int i = 0; i < configuration.rows(); i++){
                    configuration(i)=jointangles_s(i);
                }
                rns->setJointValues(configuration);

                MMM::KinematicSensorMeasurementPtr kinematicSensorMeasurement(new MMM::KinematicSensorMeasurement(timestep, configuration));
                kinematicSensor->addSensorMeasurement(kinematicSensorMeasurement);
            }
        }

        motion->saveXML(configuration->outputMotionPath);

        MMM_INFO << "--- END ---" << std::endl;
        return 0;
    }
    catch (MMM::Exception::MMMException &e) {
        MMM_ERROR << e.what() << std::endl;
        return -2;
    }
}
