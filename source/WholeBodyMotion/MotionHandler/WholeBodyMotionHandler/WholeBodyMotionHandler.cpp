#include "WholeBodyMotionHandler.h"
#include <QFileDialog>
#include <QMessageBox>
#include <MMM/Motion/MotionRecording.h>

using namespace MMM;

WholeBodyMotionHandler::WholeBodyMotionHandler(QWidget* widget) :
    MotionHandler(MotionHandlerType::GENERAL, "MMM to OpenPose"),
    searchPath(""),
    widget(widget),
    dialog(nullptr)
{
}

void WholeBodyMotionHandler::handleMotion(MotionRecordingPtr motions, std::map<std::string, VirtualRobot::RobotPtr> currentRobots) {
    if (motions && !motions->empty()) {
        if (!dialog) {
            dialog = new WholeBodyMotionHandlerDialog(widget);
            connect(dialog, &WholeBodyMotionHandlerDialog::addVisualisation, this, &WholeBodyMotionHandler::addVisualisation);
        }
        if (!dialog->isVisible()) {
            dialog->show();
        }
        dialog->openMotions(motions, currentRobots);
    }
    else MMM_ERROR << "Cannot open tool, because no motions are present!" << std::endl;
}

std::string WholeBodyMotionHandler::getName() {
    return NAME;
}
