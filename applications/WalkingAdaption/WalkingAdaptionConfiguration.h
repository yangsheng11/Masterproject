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

#pragma once

#include <filesystem>
#include <string>

#include <MMM/MMMCore.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <SimoxUtility/algorithm/string.h>

#include <VirtualRobot/RuntimeEnvironment.h>

/*!
    Configuration of MotionAdaption.
*/
class WalkingAdaptionConfiguration
{

public:
    std::string inputMotionPath;
    std::string outputMotionPath;
    bool valid = true;

    WalkingAdaptionConfiguration()
    {
    }

    bool processCommandLine(int argc, char *argv[])
    {
        VirtualRobot::RuntimeEnvironment::considerKey("inputMotion", "input mmm motion file path");
        VirtualRobot::RuntimeEnvironment::considerKey("outputMotion", "output mmm motion file path location");
        VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);

        inputMotionPath = getParameter("inputMotion", false, true);
        if (inputMotionPath.empty()) {
            std::string defaultMotionPath = std::string(PROJECT_DATA_DIR) + "Motions/MMM/walking_medium01_motion395.xml";
            std::cout << "--inputMotion not set. Using default motion from " << defaultMotionPath << std::endl;
            inputMotionPath = defaultMotionPath;
        }


        outputMotionPath = getParameter("outputMotion", false, true);
        if (outputMotionPath.empty()) {
            std::string defaultMotionPath = std::string(PROJECT_DATA_DIR) + "Motions/MMM/output.xml";
            outputMotionPath = defaultMotionPath;
        }

        VirtualRobot::RuntimeEnvironment::print();

        return valid;
    }

    void print()
    {
        MMM_INFO << "*** MotionAdaption Configuration ***" << std::endl;
        std::cout << "Input motion: " << outputMotionPath << std::endl;
    }

    std::string getParameter(const std::string &parameterName, bool required, bool isFilePath)
    {
        if (VirtualRobot::RuntimeEnvironment::hasValue(parameterName))
        {
            std::string value = VirtualRobot::RuntimeEnvironment::getValue(parameterName);

            if (isFilePath && !VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(value))
            {
                MMM_ERROR << "Invalid path: " << value << std::endl;
                valid = false;
                return std::string();
            }

            return value;
        }
        else if (required)
        {
            MMM_ERROR << "Missing parameter " << parameterName << "!" << std::endl;
            valid = false;
        }

        return std::string();
    }

};
