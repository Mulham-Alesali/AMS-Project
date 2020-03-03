

#ifdef DEBUG

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

void greifOben(){
    gripper.liftUp();
    gripper.gripOpen();
    /*while (!(gripper.isLiftMaxed() && gripper.getGripState() == 1)){
    ArUtil::sleep(100);
    }*/
    ArUtil::sleep(5000);

    gripper.liftDown();
    ArUtil::sleep(2000);
    gripper.liftStop();
    gripper.gripClose();

    gripper.liftUp();
    while (!gripper.isLiftMaxed()){
        ArUtil::sleep(100);
    }
}

void greifUnten(){
    gripper.liftUp();
    gripper.gripOpen();
    while (!(gripper.isLiftMaxed() && gripper.getGripState() == 1)){
        ArUtil::sleep(100);
    }

    gripper.liftDown();
   
    while (gripper.isLiftMaxed()){
        ArUtil::sleep(100);
    }

    gripper.liftStop();
    gripper.gripClose();

    gripper.liftUp();
    while (!gripper.isLiftMaxed()){
        ArUtil::sleep(100);
    }

}

void lassenOben(){
    gripper.gripOpen();
    ArUtil::sleep(3000);
}

void lassenUnten(){
    gripper.gripOpen();
    ArUtil::sleep(3000);
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

    greifOben();
    turnLeft90();
    lassenOben();
    turnRight90();
    greifUnten();
    turnLeft90();
    turnRight90();


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


   

    gripper.liftStop();

    //gripper.liftDown();


    //turnRight90();
    //nawid bitte hier arbeiten
    //robot.lock();
    //gripper.gripOpen();
    //robot.unlock();
    /*ArUtil::sleep(2000);
    gripper.gripStop();
   
    gripper.gripClose();
    ArUtil::sleep(1000);
    gripper.gripStop();*/




    //gripper.liftUp();
    //ArUtil::sleep(10000);
    //gripper.liftDown();
    //ArUtil::sleep(10000);
    //gripper.gripOpen();
    //ArUtil::sleep(10000);
    //gripper.gripClose();

    //turnRight90();
    //turnRight90();
    //turnLeft90();
    //turnLeft90();

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);

  return 0;
}

#else


#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <streambuf>
#include <bits.h> 
#include <sstream>

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
		cout << "greifO" << endl;
	}

	void greifU() {
		cout << "griefU" << endl;
	}

	
	void turn(direction goal) {
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

		cout << "turnleft()" << endl;
	}

	void turnRight() {

		cout << "turnRight()" << endl;
	}

	void lassO() {

		cout << "lassO" << endl;
	}

	void lassU() {

		cout << "lassU" << endl;
	}

	void move(char move[2]) {

		
			//cout << "moves" << (int)move[0] << (int)move[1] << endl;


		//turn the robot the start direction
		direction startDirection = (direction)(move[0] / 2);
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
		for (unsigned int i = 0; i < plan.size() / 2; i++) {
			//cout << plan.at(i * 2) << " " << plan.at(i * 2 + 1) << endl;
			char a[2] = { plan.at(i * 2) - '0' , plan.at(i * 2 + 1) - '0' };
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

int main() {

	system("java Main B-AC--D- BADC----");
	
	Navigator n = Navigator();

	string startStatus = "B-AC--D-";
	//string plan = "34216243";
	string plan = readFile();
	n.intit(startStatus, plan);
}

#endif // !1
