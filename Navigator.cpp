/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2015 Adept Technology, Inc.
Copyright (C) 2016 Omron Adept Technologies, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include "Aria.h"
#include <ArGripper.h>

/** @example simpleMotionCommands.cpp example showing how to connect and send
 * basic motion commands to the robot
 *
 * ARIA provides two levels of robot motion control, direct motion commands, and
 * actions. This example shows direct motion commands. See actionExample.cpp,
 * actionGroupExample.cpp, and others for examples on how to use actions.
 * Actions provide a more modular way of performing more complex motion
 * behaviors than the simple imperitive style used here.  
 *
 * See the ArRobot class documentation, as well as the overview of robot motion,
 * for more information.
 *
 * WARNING: this program does no sensing or avoiding of obstacles, the robot WILL
 * collide with any objects in the way!   Make sure the robot has about 2-3
 * meters of free space around it before starting the program.
 *
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
 */



// Adds key handler callbacks for controlling the gripper
class GripperControlHandler
{
	ArGripper* myGripper;
	ArFunctorC<GripperControlHandler> myUpCB;
	ArFunctorC<GripperControlHandler> myDownCB;
	ArFunctorC<GripperControlHandler> myOpenCB;
	ArFunctorC<GripperControlHandler> myCloseCB;
	ArFunctorC<GripperControlHandler> myStopCB;
public:
	GripperControlHandler(ArGripper* gripper) :
		myGripper(gripper),
		myUpCB(this, &GripperControlHandler::liftUp),
		myDownCB(this, &GripperControlHandler::liftDown),
		myOpenCB(this, &GripperControlHandler::open),
		myCloseCB(this, &GripperControlHandler::close),
		myStopCB(this, &GripperControlHandler::stop)
	{
	}
	void addKeyHandlers(ArRobot *robot)
	{
		ArKeyHandler *keyHandler = Aria::getKeyHandler();
		if (keyHandler == NULL)
		{
			keyHandler = new ArKeyHandler();
			Aria::setKeyHandler(keyHandler);
			robot->attachKeyHandler(keyHandler);
		}
		keyHandler->addKeyHandler(ArKeyHandler::PAGEUP, &myUpCB);
		keyHandler->addKeyHandler('u', &myUpCB);
		keyHandler->addKeyHandler(ArKeyHandler::PAGEDOWN, &myDownCB);
		keyHandler->addKeyHandler('d', &myDownCB);
		keyHandler->addKeyHandler('o', &myOpenCB);
		keyHandler->addKeyHandler('c', &myCloseCB);
		keyHandler->addKeyHandler('s', &myStopCB);
	}
	void liftUp()
	{
		ArLog::log(ArLog::Normal, "Moving gripper lift up...");
		myGripper->liftUp();
	}
	void liftDown()
	{
		ArLog::log(ArLog::Normal, "Moving gripper lift down...");
		myGripper->liftDown();
	}
	void stop()
	{
		ArLog::log(ArLog::Normal, "Stopping gripper...");
		myGripper->gripperHalt(); // stops both lift an grip
		//myGripper->liftStop(); // stops just the lift
		//myGripper->gripStop(); // stops just the gripper
	}
	void close()
	{
		ArLog::log(ArLog::Normal, "Closing gripper...");
		myGripper->gripClose();
	}
	void open()
	{
		ArLog::log(ArLog::Normal, "Opening gripper...");
		myGripper->gripOpen();
	}
};

ArRobot robot;

void turnRight90(){
	robot.lock();
	robot.setDeltaHeading(90);
	robot.unlock();
	ArUtil::sleep(3000);
}

void turnLeft90(){
	robot.lock();
	robot.setDeltaHeading(-90);
	robot.unlock();
	ArUtil::sleep(3000);
}



int main(int argc, char **argv)
{

	Aria::init();


	//gripper testing 
	ArGripper gripper(&robot);



	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();

	ArLog::log(ArLog::Terse, "WARNING: this program does no sensing or avoiding of obstacles, the robot WILL collide with any objects in the way! Make sure the robot has approximately 3 meters of free space on all sides.");

	// ArRobotConnector connects to the robot, get some initial data from it such as type and name,
	// and then loads parameter files for this robot.
	ArRobotConnector robotConnector(&parser, &robot);
	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
		if (parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
			return 1;
		}
	}
	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	ArLog::log(ArLog::Normal, "simpleMotionCommands: Connected.");

	// Start the robot processing cycle running in the background.
	// True parameter means that if the connection is lost, then the 
	// run loop ends.
	robot.runAsync(true);

	/* Motore einschalten */
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	gripper.gripOpen();
	ArUtil::sleep(10000);
	gripper.gripClose();

	turnRight90();
	turnRight90();
	turnLeft90();
	turnLeft90();

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
