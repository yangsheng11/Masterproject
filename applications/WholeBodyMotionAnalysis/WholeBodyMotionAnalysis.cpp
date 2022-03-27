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

#include "WholeBodyMotionAnalysisConfiguration.h"

#include <string>
#include <vector>
#include <tuple>
#include<fstream>
#include <MMM/Motion/MotionReaderXML.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Dynamics/Dynamics.h>
#include <WholeBodyMotion/WholeBodyMotionLib/finite_difference.hpp>
// #include<boost/math/differentiation/finite_difference.hpp>
#include<iomanip>
using namespace  boost::math::differentiation;
using namespace MMM;
using namespace std;
int main(int argc, char *argv[]) {
    MMM_INFO << "--- Whole Body Motion Analysis ---" << std::endl;

    WholeBodyMotionAnalysisConfiguration *configuration = new WholeBodyMotionAnalysisConfiguration();
    if (!configuration->processCommandLine(argc,argv)) {
        MMM_ERROR << "Could not process command line, aborting." << std::endl;
        return -1;
    }



    configuration->print();

    try {
        bool onlyMMMmotion = true;
        MMM::MotionReaderXMLPtr motionReader(new MMM::MotionReaderXML(true, onlyMMMmotion));
        MMM_INFO << "Reading motion file '" << configuration->inputMotionPath << "'!" << std::endl;
        MMM::MotionRecordingPtr motions = motionReader->loadMotionRecording(configuration->inputMotionPath); // can also load from directory

        std::string robotNodeSetName = "LeftArm-7dof";
        bool logging = false;
        std::ofstream fp("acceleration-torques.dat");
        fp<<"timestep"<<" "<<"jointacceleration"<<" "<<"jointtorques"<<std::endl;

        for (auto motion : *motions) {
            if (!motion->isReferenceModelMotion()) continue; // do only for mmm reference motion

            std::vector<float> timesteps = motion->getTimesteps();
            VirtualRobot::RobotPtr model = motion->getModel()->cloneScaling();
            VirtualRobot::RobotNodeSetPtr rns = model->getRobotNodeSet(robotNodeSetName);
            VirtualRobot::DynamicsPtr dynamics_model(new VirtualRobot::Dynamics(rns, model->getRobotNodeSet("All"), logging));

            // Print the inertia matrix for each timestep for the robot node set
            for (float timestep : timesteps) {
                motion->initializeModel(model, timestep);
                Eigen::Vector3f position=model->getRobotNodeSet("LeftArm-7dof")->getNode(6)->getGlobalPosition();
                Eigen::Matrix3f orientation=model->getRobotNodeSet("LeftArm-7dof")->getNode(6)->getGlobalOrientation();
                Eigen::VectorXd jointAngles_rns = rns->getJointValuesEigen().cast<double>();
                Eigen::MatrixXd inertia_matrix = dynamics_model->getInertiaMatrix(jointAngles_rns, false);
                double theta=0.01,timecopy,timecopy1;
                timecopy=timestep;timecopy1=timestep;
                timecopy+=theta;timecopy1+=2*theta;
                motion->initializeModel(model,timecopy);
                Eigen::VectorXd jointAngles_next = rns->getJointValuesEigen().cast<double>(),jointVelocity,jointacceleration;
                motion->initializeModel(model,timecopy1);
                Eigen::VectorXd jointAngles_last = rns->getJointValuesEigen().cast<double>();
                jointVelocity=(jointAngles_next-jointAngles_rns)/theta;
                jointacceleration=(jointAngles_last+jointAngles_rns-2*jointAngles_next)/(theta*theta);
                Eigen::VectorXd jointTorques = inertia_matrix * jointacceleration;


                fp<<timestep<<"        "<<jointacceleration[0]<<"          "<<jointTorques[0]<<"\n"<<"          "<<jointacceleration[1]
                 <<"          "<<jointTorques[1]<<"\n"
                <<"          "<<jointacceleration[2]<<"          "<<jointTorques[2]<<"\n"
                <<"          "<<jointacceleration[3]<<"          "<<jointTorques[3]<<"\n"
                <<"          "<<jointacceleration[4]<<"          "<<jointTorques[4]<<"\n"
                <<"          "<<jointacceleration[5]<<"          "<<jointTorques[5]<<"\n"
                <<"          "<<jointacceleration[6]<<"          "<<jointTorques[6]<<"\n"
                <<std::endl;

               std::cout << "Timestep: " << timestep<<"\n"<<jointAngles_rns<<"\n"<<position<<"\n"<<orientation<<"\n"<< std::endl;
            }
        }
        fp.close();
        //std::cout<<motions->size()<<std::endl;
        MMM_INFO << "--- END ---" << std::endl;
        return 0;
    }
    catch (MMM::Exception::MMMException &e) {
        MMM_ERROR << e.what() << std::endl;
        return -2;
    }
}
