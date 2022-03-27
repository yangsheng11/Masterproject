#include "WholeBodyMotionHandlerDialog.h"
#include "ui_WholeBodyMotionHandlerDialog.h"

#include <MMM/Exceptions.h>
#include <MMM/Motion/MotionRecording.h>
#include <MMM/Model/ModelReaderXML.h>
#include <QFileDialog>
#include <QCheckBox>
#include <QMessageBox>
#include <VirtualRobot/Robot.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoUnits.h>
#include <MMM/Motion/Plugin/ModelPosePlugin/ModelPoseSensor.h>
#include <MMM/Motion/Plugin/ModelPosePlugin/ModelPoseSensorMeasurement.h>
#include <MMM/Motion/Plugin/KinematicPlugin/KinematicSensorMeasurement.h>
#include <MMM/Motion/Plugin/KinematicPlugin/KinematicSensor.h>
#include <SimoxUtility/math/convert.h>
#include <VirtualRobot/Visualization/VisualizationNode.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/TriMeshUtils.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/SbVec3f.h>
#include <QPushButton>
#include <QFileDialog>
#include <set>
#include <QDoubleSpinBox>

WholeBodyMotionHandlerDialog::WholeBodyMotionHandlerDialog(QWidget* parent) :
    QDialog(parent),
    ui(new Ui::WholeBodyMotionHandlerDialog),
    motions(nullptr)
{
    ui->setupUi(this);
}

WholeBodyMotionHandlerDialog::~WholeBodyMotionHandlerDialog() {
    delete ui;
}

void WholeBodyMotionHandlerDialog::jumpTo(float timestep) {
    SoSeparator* visualization = new SoSeparator();
    // todo update vis

    // current robots should already be set to the current pose

    emit addVisualisation(visualization);

}

void WholeBodyMotionHandlerDialog::openMotions(MMM::MotionRecordingPtr motions, const std::map<std::string, VirtualRobot::RobotPtr> &currentRobots) {
    this->motions = motions;
    this->currentRobots = currentRobots;

    // example for retrieving modelpose and kinematics if needed
//    auto motion = motions->getReferenceModelMotion(); // retrieves the mmm model
//    if (motion) {
//        auto modelPoseSensor = motion->getSensorByType<MMM::ModelPoseSensor>();
//        auto kinematicSensor = MMM::KinematicSensor::join(motion->getSensorsByType<MMM::KinematicSensor>());
//        for (auto timestep : modelPoseSensor->getTimesteps()) {
//            auto modelPoseSensorMeasurement = modelPoseSensor->getDerivedMeasurement(timestep);
//            auto kinematicSensorMeasurement = kinematicSensor->getDerivedMeasurement(timestep);
//            if (modelPoseSensorMeasurement && kinematicSensorMeasurement) {
//                // ...
//            }
//        }
//    }
}



