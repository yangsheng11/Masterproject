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
* @copyright  2021 High Performance Humanoid Technologies (H2T), Karlsruhe, Germany
*
*/

#pragma once

#include <MMM/Viewer/MotionHandler.h>
#include "WholeBodyMotionHandlerDialog.h"

namespace MMM
{

class WholeBodyMotionHandler : public MotionHandler
{
    Q_OBJECT

public:
    WholeBodyMotionHandler(QWidget* widget);

    void handleMotion(MotionRecordingPtr motions, std::map<std::string, VirtualRobot::RobotPtr> currentRobots);

    virtual std::string getName();

    virtual void timestepChanged(float timestep) {
        if (dialog && dialog->isVisible()) dialog->jumpTo(timestep);
    }

    virtual void motionsOpened(bool opened, MMM::MotionRecordingPtr motions, std::map<std::string, VirtualRobot::RobotPtr> currentRobots) {
        if (dialog && dialog->isVisible() && opened) dialog->openMotions(motions, currentRobots);
    }

    static constexpr const char* NAME = "WholeBodyMotionHandler";

private:
    std::string searchPath;
    QWidget* widget;
    WholeBodyMotionHandlerDialog* dialog;
};

}
