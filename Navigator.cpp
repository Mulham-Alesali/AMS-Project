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
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <streambuf>
#include <bits.h>
#include <sstream>
#include <istream>
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
        keyHandler->addKeyHandler(ArKeyHandler:AGEUP, &myUpCB);
        keyHandler->addKeyHandler('u', &myUpCB);
        keyHandler->addKeyHandler(ArKeyHandler:AGEDOWN, &myDownCB);
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
ArGripper gripper(&robot);



using namespace std;

enum direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class Navigator {

public:
    direction currentDirection = NORTH;

    string currentStatus;

    void intit(string currentState, string plan) {
        this->currentStatus = currentStatus;
        solve(plan);
    }

    void greifO() {
        cout << "greif oben" << endl;
        gripper.liftUp();
        gripper.gripOpen();
        /*while (!(gripper.isLiftMaxed() && gripper.getGripState() == 1)){
        ArUtil::sleep(100);
        }*/
        ArUtil::sleep(3000);

        gripper.liftDown();
        ArUtil::sleep(2000);
        gripper.liftStop();
        gripper.gripClose();
        ArUtil::sleep(3000);
        gripper.liftUp();
        ArUtil::sleep(3000);
        while (!gripper.isLiftMaxed()){
            ArUtil::sleep(100);
        }

    }

    void greifU() {

        cout << "greif Unten" << endl;
        gripper.liftUp();
        gripper.gripOpen();
        ArUtil::sleep(3000);
   

        gripper.liftDown();
        ArUtil::sleep(3000);
       

        gripper.liftStop();
        gripper.gripClose();
        ArUtil::sleep(3000);
        gripper.liftUp();
        ArUtil::sleep(3000);
       
       
        cout << "end of greifunten" << endl;
       
    }


   
    void turn(direction goal) {
        cout << "turn to goal" << (goal) << endl;
        //cout << this->currentDirection << " : " << goal << endl;
        int turnNumber = goal - this->currentDirection;
        //cout << "turn number: " << turnNumber << endl;
        if (turnNumber > 0) {
            if (turnNumber > 2) {
                turnLeft();
            }
            else {
                for (int i = 0; i < turnNumber; i++) {
                    turnRight();
                }
            }
        }
        else if (turnNumber < 0){
            turnNumber = abs(turnNumber);
            if (turnNumber > 2) {
                turnRight();
            }
            else {
                for (int i = 0; i < turnNumber; i++) {
                    turnLeft();
                }
            }

        }



        this->currentDirection = goal;
        //cout << this->currentDirection << " : " << goal << endl;
    }


    void turnLeft() {
        cout << "start turning left" << endl;
        ArUtil::sleep(3000);
        robot.lock();
        robot.setDeltaHeading(90);
        robot.unlock();
        ArUtil::sleep(3000);
        cout << "end of turning left" << endl;
    }

    void turnRight() {
        cout << "start turning right" << endl;
        robot.lock();
        robot.setDeltaHeading(-90);
        robot.unlock();
        ArUtil::sleep(3000);
        cout << "end of turning right" << endl;
    }

    void lassO() {
        cout << "start of lassing oben" << endl;
        gripper.gripOpen();
        ArUtil::sleep(3000);
        cout << "end of lassing oben" << endl;
    }

    void lassU() {
        cout << "start of lassing unten" << endl;
        gripper.gripOpen();
        ArUtil::sleep(3000);
        cout << "end of lassing unten" << endl;
    }



    void move(char move[2]) {


        //cout << "moves" << (int)move[0] << (int)move[1] << endl;


        //turn the robot the start direction
        direction startDirection = (direction)(move[0] / 2);
        cout << "turn to start direction" << " start direction " << startDirection << endl;
        ArUtil::sleep(5000);
        turn(startDirection);


        //carry the box
        //is the box up or down
        if (move[0] % 2 == 1) {
            greifO();
        }
        else {
            greifU();
        }

        //find the goal direction and turn the robot to it
        direction goalDirection = (direction)(move[1] / 2);
        cout << "turn to goal direction" << endl;
        turn(goalDirection);

        if (move[1] % 2 == 1) {
            lassO();
        }
        else {
            lassU();
        }


    }


    void solve(string plan) {
        //cout << plan.size() << endl;
        cout << "solve der Plan" << endl;
        for (unsigned int i = 0; i < plan.size() / 2; i++) {
            //cout << plan.at(i * 2) << " " << plan.at(i * 2 + 1) << endl;
            char a[2] = { plan.at(i * 2) - '0', plan.at(i * 2 + 1) - '0' };
            cout << "move befehl: " << (char)(a[0] + '0') << (char)(a[1] + '0') << endl;
            move(a);
        }

    }

};


string readFile()
{
    stringstream str;
    ifstream stream("FileWriter.txt");
    if (stream.is_open())
    {
        while (stream.peek() != EOF)
        {
            str << (char)stream.get();
        }
        stream.close();
        return str.str();
    }
}


int main(int argc, char **argv)
{
   



    Aria::init();


    //gripper testing



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
            //return 1;
        }
    }
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        //return 1;
    }

    ArLog::log(ArLog::Normal, "simpleMotionCommands: Connected.");

    // Start the robot processing cycle running in the background.
    // True parameter means that if the connection is lost, then the
    // run loop ends.
   
    robot.runAsync(true);
   
    robot.lock();
    robot.enableMotors();
    robot.unlock();

        cout << "geben Sie bitte den Startzustand ein!\n z.b. B-AC--D-\n" << endl;
        string s;
        cin >> s;
        int count = 0;
    while (true){

        string e;
        cout << "geben Sie bitte den Endzustand ein!\n z.b. BADC---- \n" << endl;
        cin >> e;
       
        string commandString = "java Main " + s + " " + e;
        const char * command = commandString.c_str();
        //system("java Main B-AC--D- BADC----");
        system(command);
        //b-a-c
        Navigator n = Navigator();
        string startStatus = "B-AC--D-";
        //string plan = "34216243";
        string plan = readFile();
        cout << "plan: " << plan << endl;
        n.intit(startStatus, plan);
        n.turn(NORTH);
        s = e;
    }


    /* Motore einschalten
    gripper.liftUp();
    gripper.gripOpen();
    //ArUtil::sleep(10000);
    ArUtil::sleep(7000);
    gripper.liftDown();
    ArUtil::sleep(2000);
    gripper.liftStop();
    gripper.gripClose();

    ArUtil::sleep(5000);
    gripper.liftUp();
    turnRight90();
    gripper.gripOpen();
    ArUtil::sleep(5000);

    turnLeft90();
    gripper.liftDown();
    ArUtil::sleep(5000);
    gripper.gripClose();
    ArUtil::sleep(2000);

    gripper.liftUp();
    ArUtil::sleep(5000);
    turnRight90();
    gripper.gripOpen();
    ArUtil::sleep(3000);
    turnLeft90();

   
    */


   



  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
