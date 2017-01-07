#include <libplayerc++/playerc++.h>

#include <cmath>
#include <math.h>
#include <iostream>
#include <queue>
#include <string>
#include <fstream>

using namespace PlayerCc;

/**
 *MoveStraight // (Move Straight with init taking distance d)
 *Turn() // Turn fixed angle (int taking the angle to rotate a a)
 *Random // init is empty might reset motion
 *MovePosition // Move to a position (position provided by init)
 *MoveNetwork // Move on way point network (with int taking file name with positions)
 *Wait // Wait at current position (trivial do nothing)
 */
#define MAX_ROTATION (3.1415926535897932 / 6)
//////////////////////////////Collision///////////////////////////////////////////////
#define RANGER_FRONT 0
#define RANGER_LEFT 1
#define RANGER_RIGHT 2
#define RANGER_LEFT_90 3
#define RANGER_RIGHT_90 4

#define DANGER_DIST_RATIO 0.05
#define CRATICAL_DIST 0.5
#define PI 3.141592653589793238462643383279502884197169399375105

class Collision
{
        public:
                Collision(Position2dProxy* pp, RangerProxy* lp); 

                bool CheckMovement(double speed, double rotation);

                // Give a direction that has "free space to move into
                double DirectionOfFreeSpace(void); 


        private:
                Position2dProxy* pos_proxy;
                RangerProxy*  ranger;
				double cached_value;

				
};

Collision::Collision(Position2dProxy* pp, RangerProxy* lp)
{
	pos_proxy = pp;
	ranger = lp;
	cached_value = PI/6;
	ranger->RequestGeom();
    ranger->RequestConfigure();
}

bool Collision::CheckMovement(double speed, double rotation)
{
	const int  num_ray=ranger->GetElementCount();
	bool result=true;
	const double safe_dist=.65;
	for(int i = 0;i<num_ray;i++){
		if((*ranger)[i]<safe_dist){
			result=false;
			printf("Space Violation \n");
			break;
		}
	}


    return(result);
}

double Collision::DirectionOfFreeSpace(void)
{
	double dist_front, dist_left, dist_right, dist_left_90, dist_right_90;
	dist_front = ranger->GetRange(RANGER_FRONT);
	dist_left = ranger->GetRange(RANGER_LEFT);
	dist_right = ranger->GetRange(RANGER_RIGHT);
	dist_right_90 = ranger->GetRange(RANGER_RIGHT_90);
	dist_left_90 = ranger->GetRange(RANGER_LEFT_90);
	//std::cout << "rotate debug " << dist_front << " " << dist_left << " " << dist_right << std::endl;
	if (dist_front < CRATICAL_DIST && dist_left < CRATICAL_DIST && dist_right < CRATICAL_DIST)
	{
		if(dist_right_90 > dist_left_90) return PI/2.0;
		else 							 return -PI/2.0;
	}
	
	else if ((dist_front < dist_left) && (dist_right < dist_left))
	{
		cached_value = -PI/3.0;
		return -PI/3.0;
	}
	else if ((dist_front < dist_right) && (dist_left < dist_right))
	{
		cached_value = PI/3.0;
		return PI/3.0;
	}
	else return cached_value;
}

//////////////////////////////Position////////////////////////////////////////////////
class Position {
	public:
		Position(double x, double y);
		Position(void);

		double getX(void);
		double getY(void);

	private:
		double x;//x position
		double y;//y position
		
};

Position::Position(double x0, double y0) 
{
	x = x0;
	y = y0;
}

Position::Position(void) 
{
	x = 0.0;
	y = 0.0;
}

double Position::getX(void) 
{
	return x;
}

double Position::getY(void) 
{
	return y;
}


//////////////////////////////Behaviors//////////////////////////////////////////////
class Behaviors
{

        public:
                Behaviors(Position2dProxy* pp); // Constructor (Default constructor is private)
                virtual void Init(void){}; // Overwritten by childeren to given them information
                virtual bool Tick(void) =0; //  Called every frame, true is Behavior complete
                virtual void Resume(void){}; // Resume after other behavior was done
                double GetIntendedSpeed(){return(speed);};
                double GetIntendedRotation(){return(rotation);};

                void DoMove(void); //Perform the intended move (not overwriten by childen !!)

        protected:
                double speed,rotation; // The intended speed in this frame
                Position2dProxy* posProxy; // The position proxy to obain information !!

        private:
                Behaviors(){}; // Private !!

};

Behaviors::Behaviors (Position2dProxy* pp) 
{
	posProxy = pp;
	speed = 0.0;
	rotation = 0.0;
}

void Behaviors::DoMove(void) 
{
    //printf("MOVE %f %f \n", speed, rotation);
	posProxy -> SetSpeed(speed, rotation);
}

/////////////////////////Wait////////////////////////////////////////////////

class  Wait : public Behaviors
{

	public:
		Wait(Position2dProxy* pp);
		bool Tick(void);
};

Wait::Wait(Position2dProxy* pp):Behaviors(pp) {return;}

bool Wait::Tick(void)
{
	return true;
}

///////////////////////Turn///////////////////////////////////////////////

#define ERROR_ANGLE 0.1
#define ANGLE_RATIO 2.0
#define PI_TWO (2*3.141592653589793238462643383279502884197169399375105)

class Turn : public Behaviors
{
	public:
		Turn(Position2dProxy* pp);
		void Init(double a);
		bool Tick(void);

		void Resume(void);
		
	protected:
		
	private:
		double initAngle;
		double desiredAngle;
		
		double getRemainingAngle(void);
};


Turn::Turn(Position2dProxy* pp):Behaviors(pp)
{
	initAngle = 0.0;
	desiredAngle = 0.0;
}

void Turn::Init(double a) 
{
	initAngle = posProxy->GetYaw();
	desiredAngle = initAngle + a;
	if(desiredAngle > PI) desiredAngle =desiredAngle - PI_TWO;
	//std::cout << "Turn.Init(): called; desiredA: " << desiredAngle << "; initAngle: "<< initAngle << std::endl;
}

double Turn::getRemainingAngle(void) 
{
	double angleDiff = desiredAngle - posProxy->GetYaw();
	//std::cout << "Turn.getRemainingAngle(): called; " << "currA: " << posProxy->GetYaw() << "ADiff: " << angleDiff << std::endl;
	if (angleDiff > PI) 	   return angleDiff - (2 * PI);
	else if (angleDiff < -PI)  return angleDiff + (2 * PI);
	else                       return angleDiff;
}

bool Turn::Tick(void) 
{
	if (fabs(getRemainingAngle()) <= ERROR_ANGLE) 
	{
		//std::cout << "Turn.Tick(): called; Done! rotation = 0 " << std::endl;
		rotation = 0.0;
		return true;
	}
	rotation = getRemainingAngle() / ANGLE_RATIO;
	//std::cout << "Turn.Tick(): called; doing rotation = " << rotation << std::endl;
	return false;
}

void Turn::Resume(void) 
{
	//std::cout << "Turn.Resume(): called" << std::endl;
	if (abs(getRemainingAngle()) <= ERROR_ANGLE)
	{
		rotation = getRemainingAngle() / ANGLE_RATIO;
	}
	else
	{
		rotation = 0.0;
	} 
}


///////////////////MoveStraight///////////////////////////////////////////////

class MoveStraight:public Behaviors
{
	public:
		MoveStraight(Position2dProxy* pp);
		void Init(double d); // Distance to Move
		bool Tick(void);
		void Resume(void);

	protected:
	
		
	private:
		Position* initPos;
		double dist;
		bool initCheck;
		
		double getRemainDist(void);
};


#define sq(x) ((x)*(x))
#define ERROR 0.3
#define RATIO 1.3

MoveStraight::MoveStraight(Position2dProxy* pp):Behaviors(pp)
{
	initPos = new Position(0.0, 0.0);
}

void MoveStraight::Init(double d) 
{	
	initCheck = true;
	initPos = new Position(posProxy->GetXPos(), posProxy->GetYPos());
	dist = d;
	//std::cout << "MoveStraight.Init(): called; Dist:" << dist << "; initX,Y: "<< initPos->getX() << " " << initPos->getY() << std::endl;
}

double MoveStraight::getRemainDist(void) 
{
	double out = dist - sqrt(sq(posProxy->GetXPos() - initPos->getX()) +
				 sq(posProxy->GetYPos() - initPos->getY()));
	//std::cout << "MoveStraight.getRemainDist() remain: " << out << std::endl;
	return out;
}


bool MoveStraight::Tick(void) 
{
	if(initCheck) 
	{
		initCheck = false;
		initPos = new Position(posProxy->GetXPos(), posProxy->GetYPos());
		//std::cout << "MoveStraight.Tick(): called; initX,Y changed to: "<< initPos->getX() << " " << initPos->getY() << std::endl;
	}
	if (getRemainDist() <= ERROR) 
	{
		speed = 0.0;
		//std::cout << "MoveStraight.Tick() Finished! zero speed" << std::endl;
		return true;
	}
	speed = getRemainDist() / RATIO;
	//std::cout << "MoveStraight.Tick() " << "Speed set to be " << speed << std::endl;
	return false;
}

void MoveStraight::Resume(void) 
{
	//std::cout << "MoveStraight.Resume()" << std::endl;
	if (getRemainDist() >= dist - ERROR)
	{
		speed = getRemainDist() / RATIO;
	}
	else
	{
		speed = 0.0;
	} 
}


/////////////////////////Random/////////////////////////////////////
class Random:public Behaviors
{
	public:
		Random(Position2dProxy* pp);
		bool Tick(void);
		void Init(void);

	protected:
		
		
	private:
		

};


#define FILTER_RATIO 0.25

Random::Random (Position2dProxy* pp):Behaviors(pp) {return;}

void Random::Init(void)
{
	srand(time(NULL));
	speed = 2.0;
}

bool Random::Tick(void)
{	
	double x_rand = rand() % 200;
    double direction = rand() % 2;
    //Using posProxy->GetYaw() to get the current yaw of the robot
    if (direction <= 0)
		rotation = (1 - FILTER_RATIO) * posProxy->GetYaw() + FILTER_RATIO * x_rand;
    else
        rotation = (1 - FILTER_RATIO) * posProxy->GetYaw() - FILTER_RATIO * x_rand;

    return false;
}

///////////////////MovePosition///////////////////////////////////
class MovePosition : public Behaviors
{
	public:
		MovePosition(Position2dProxy* pp);
		void Init(double x, double y);
		bool Tick(void);

		void Resume(void);
		
	protected:
		
		
	private:
		Position* des; //Destination
};

#define one_tenth_pi 0.3141592653589793238462643383279502884197169399375105
#define pi 3.141592653589793238462643383279502884197169399375105

MovePosition::MovePosition (Position2dProxy* pp):Behaviors(pp)
{
	des = new Position(0.0, 0.0);
}

void MovePosition::Init(double x, double y)
{
	des = new Position(x, y);
}

bool MovePosition::Tick(void)
{
	double curr_yaw = posProxy->GetYaw(), 
		 curr_x = posProxy->GetXPos(),
		 curr_y = posProxy->GetYPos();
	
	double dist = sqrt((des->getX() - curr_x) * (des->getX() - curr_x) +
				 (des->getY() - curr_y) * (des->getY() - curr_y));

	if (dist <= ERROR)
	{
		speed = 0.0;
		rotation = 0.0;
		return true;
	}

	speed = dist / RATIO;

	double desired_angle = atan2(des->getY() - curr_y, des->getX() - curr_x);
	
	rotation = desired_angle - curr_yaw;

    // Limit the angle to be -pi ~ pi
	if (rotation > pi) rotation -= 2 * pi;
	else if (rotation < -pi) rotation += 2 * pi;

	if (rotation > MAX_ROTATION) rotation = MAX_ROTATION;
	else if (rotation < -MAX_ROTATION) rotation = -MAX_ROTATION;

	return false;
}

void MovePosition::Resume(void)
{
	Tick();
}

///////////////////MoveNetwork///////////////////////////////////
class MoveNetwork : public Behaviors
{
public:
		MoveNetwork(Position2dProxy* pp);
		void Init(const char* FILE);
		bool Tick(void);

		void Resume(void);
		
	protected:
		
		
	private:
		std::queue<Position*>* posQueue;
		bool moveToPos(Position des);
};
#define ERROR_DIST 0.1

MoveNetwork::MoveNetwork (Position2dProxy* pp):Behaviors(pp)
{
	posQueue = new std::queue<Position*>;
}

void MoveNetwork::Init(const char* FILE)
{
	double a1, a2;

	std::ifstream myfile;
	myfile.open(FILE);
		
	while (myfile >> a1 >> a2)
	{
		posQueue->push(new Position(a1, a2));
	}

	//std::cout<< "MoveNetwork done init()" << std::endl;
}

bool MoveNetwork::moveToPos(Position des) 
{
	const double curr_yaw = posProxy->GetYaw(), 
				 curr_x = posProxy->GetXPos(),
				 curr_y = posProxy->GetYPos();
	
	double dist = sqrt((des.getX() - curr_x) * (des.getX() - curr_x) +
				 (des.getY() - curr_y) * (des.getY() - curr_y));

	double desired_angle = atan2(des.getY() - curr_y, des.getX() - curr_x);
	
	double angle_diff = desired_angle - curr_yaw;

    // Limit the angle to be -pi ~ pi
	if (angle_diff > pi) angle_diff -= 2 * pi;
	else if (angle_diff < -pi) angle_diff += 2 * pi;

	double yawSpeed = angle_diff; // speed_scale;
	double aspeed   = dist / RATIO;
	std::cout << " Destination X: " << des.getX() << " Y: " << des.getY() << std::endl;

	if (dist <= ERROR_DIST) {
		speed = 0.0;
		rotation = 0.0;
		//std::cout << "Arrived..." << std::endl;
		return true;
	} else {
		speed = aspeed;
		rotation = yawSpeed;

		if (rotation > MAX_ROTATION) rotation = MAX_ROTATION;
		else if (rotation < -MAX_ROTATION) rotation = -MAX_ROTATION;
		return false;
	}
}

bool MoveNetwork::Tick(void)
{
	if(moveToPos(*posQueue->front())) 
	{
		
		posQueue->push(posQueue->front());
		posQueue->pop();
	}
	return false;
}



void MoveNetwork::Resume(void)
{
	Tick();
}
//////////////////////////////////Robot////////////////////////////////////////
class Robot
{
        public:
                Robot(PlayerClient* client, int index); // Open the robot with a specific index setup everything,

				Position* GetPos(void); // Return current position of this robot
                //! Get Robot to preapre for move
                void Tick(void); // Give Computational time

                //! Initialize the classes, Call before first tick !!!
                void Init(void);

                virtual Behaviors* ChangeBevaviour(void) = 0; //! Overwrite this to create bevavior
                virtual Behaviors* ChangeCollisionBevaviour(void) = 0;//! Overwrite this to create bevavior       


        protected:
                Position2dProxy* pos_proxy;
                RangerProxy*  ranger;
				Behaviors*  current_behavior;
				Collision * colli;
				int mode;
};

Robot::Robot(PlayerClient* client, int index)
{
	pos_proxy = new Position2dProxy(client, index);
	ranger = new RangerProxy(client, index);
	std::cout << client << std::endl;
	colli = new Collision(pos_proxy, ranger);
	
    pos_proxy->SetMotorEnable (true);
}

void Robot::Init(void)
{
	current_behavior = ChangeBevaviour();
	current_behavior -> Init();
	mode = 0;
}

Position* Robot::GetPos(void)
{
	return new Position(pos_proxy->GetXPos(), pos_proxy->GetYPos());
}


void Robot::Tick(void)
{
	if(current_behavior->Tick())
	{
	// DONE with Behaviors
	// change bebavior (I used a queue of Behaviors for testing)
	// Note call Resume or init on Behaviors gefore using it.
		current_behavior = ChangeBevaviour();
	}
	else
	{
	// REMEMBER HERE WE WILL (next lab) CHECK IF INTENDED MOVE IS OK
	// USING behav-> GetIntendedSpeed() and
	// behav->GetIntendedRotation()
			
		if (colli -> CheckMovement(current_behavior -> GetIntendedSpeed(), current_behavior -> GetIntendedRotation()))
		{
			if (mode = 1) 
			{
				current_behavior = ChangeBevaviour();
				mode = 0;
			}
			current_behavior->DoMove();
		}
		else
		{
			//std::cout << "avoid collision "<< colli -> DirectionOfFreeSpace() << std::endl;

			current_behavior = ChangeCollisionBevaviour();
			current_behavior->DoMove();
			mode = 1;
		}
		
	}
}



//////////////////Robot_Path/////////////////////////
class Robot_Path : public Robot
{
	public:
		Robot_Path(PlayerClient* client, int index);
		~Robot_Path();
		void Init(void);
		Behaviors* ChangeBevaviour(void); //! create bevavior
        Behaviors* ChangeCollisionBevaviour(void);//! create bevavior 

	private:
		Behaviors* turn; //= new Turn(&pp);
		Behaviors* moveNetwork;// = new MoveNetwork(&pp);
		
};

Robot_Path::Robot_Path(PlayerClient* client, int index):Robot(client, index)
{
	turn = new Turn(pos_proxy);
	moveNetwork = new MoveNetwork(pos_proxy);
}

Robot_Path::~Robot_Path()
{
	delete turn;
	delete moveNetwork;
}

void Robot_Path::Init(void)
{
	
	current_behavior = ChangeBevaviour();
	((MoveNetwork*) current_behavior) -> Init("data.txt");
	mode = 0;
}

Behaviors* Robot_Path::ChangeBevaviour(void)
{
	return (moveNetwork);
}

Behaviors* Robot_Path::ChangeCollisionBevaviour(void)
{
	((Turn*) turn) -> Init(colli -> DirectionOfFreeSpace());
	std::cout << "find collision leader"<< std::endl;
	return (turn);
}
////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////Move_Position_Robot//////////////////////////////////////
class Move_Position_Robot : public Robot
{
	public:
		Move_Position_Robot(PlayerClient* client, int index);
		~Move_Position_Robot();
		void Init(void);
		Behaviors* ChangeBevaviour(void); //! create bevavior
        Behaviors* ChangeCollisionBevaviour(void);//! create bevavior 
		void InitPos(Position* des);

	private:
		Behaviors* turn; //= new Turn(&pp);
		Behaviors* movePosition;// = new MovePosition(&pp);
		
};

Move_Position_Robot::Move_Position_Robot(PlayerClient* client, int index):Robot(client, index)
{
	turn = new Turn(pos_proxy);
	movePosition = new MovePosition(pos_proxy);
}

Move_Position_Robot::~Move_Position_Robot()
{
	delete turn;
	delete movePosition;
}

void Move_Position_Robot::Init(void)
{
	
	current_behavior = ChangeBevaviour();
	((MovePosition*) current_behavior) -> Init(0, 0);
	mode = 0;
}

void Move_Position_Robot::InitPos(Position* des)
{
	((MovePosition*) current_behavior) -> Init(des -> getX(), des -> getY());
}

Behaviors* Move_Position_Robot::ChangeBevaviour(void)
{
	return (movePosition);
}

Behaviors* Move_Position_Robot::ChangeCollisionBevaviour(void)
{
	((Turn*) turn) -> Init(colli -> DirectionOfFreeSpace());
	std::cout << "find collision follower"<< std::endl;
	return (turn);
}

/*------------------------------------------------------------------------*/

std::string  gHostname(PlayerCc::PLAYER_HOSTNAME);

int main(int argc, char **argv)
{
	try{
	
		PlayerClient client(gHostname, 63216); // Open connection to server
		std::cout << client << std::endl;
		

		Robot_Path *lead;
		Position *lead_pos;
		Position *lead_pos1;
		Move_Position_Robot *follower1;
		Move_Position_Robot *follower2;

		lead = new Robot_Path(&client,0); // Create robot at index 0 (later we open several)
		follower1 = new Move_Position_Robot(&client,1); // Create robot at index 0 (later we open several)
		follower2 = new Move_Position_Robot(&client,2); // Create robot at index 0 (later we open several)

		lead -> Init();
		follower1 -> Init();
		follower2 -> Init();

		client.Read(); // First ready to get data flowing !!

		for(;;){
			lead_pos = lead->GetPos();
			lead_pos1 = follower1->GetPos();

			client.Read(); // Read data for all robots
			lead->Tick(); // Compute movement for robot
			
			follower1 -> InitPos(lead_pos);
			follower1->Tick();

			follower2 -> InitPos(lead_pos1);
			follower2->Tick();
		}

	} catch (PlayerCc::PlayerError & e){
		std::cerr << e << std::endl;
		return -1;
	}
}

