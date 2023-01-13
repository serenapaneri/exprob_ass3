/** @package exprob_ass3
*
* \file simulation.cpp
* \brief this node generates and pubblish the hints of the game.
*
* \author Serena Paneri
* \version 1.0
* \date 13/01/2023
*
* \details
*
* Subscribes to: <BR>
*     None
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*     /oracle_hint
*     /oracle_solution
*
* Client Services: <BR>
*     None
*
* Action Services: <BR>
*     None
*
* Description: <BR>
*  
*/

#include <ros/ros.h>
#include <exprob_ass2/ErlOracle.h>
#include <exprob_ass2/Oracle.h>
#include <exprob_ass3/Marker.h>

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

// publisher for oracle_hint
ros::Publisher oracle_pub;

// coordinates of the 4 sources of hints in the environment
double markx[4];
double marky[4];
double markz[4];

// x and y coordinates of the last marker visited
double lastmarkx = 0.0;
double lastmarky = 0.0;

// constant arrays containing all the types of individuals
const std::string key[3] = {"who", "what", "where"};
// constant arrays containing all the individuals of the game
const std::string person[6] = {"MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"};
const std::string object[6] = {"candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"};
const std::string place[9] = {"conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"}; 

int uIDs[3]={-1,-1};
// winning ID
int winID = -1;
 
int markerID[30]; 
exprob_ass2::ErlOracle oracle_msgs[30];
 

/**
* \brief provides a service to check the winning ID
* \param req: request from the client
* \param res: response from the service oracle_solution
* \return true
*
* This function checks if the ID of the current hypothesis is the winning one.
*/
bool oracleService(exprob_ass2::Oracle::Request &req, exprob_ass2::Oracle::Response &res)
	{
		res.ID = winID;
		return true;
	}

bool oracleCallback(exp_assignment3::Marker::Request &req, exp_assignment3::Marker::Response &res)
{
	res.oracle_hint = oracle_msgs[req.markerId-11];
	return true;
} 

int main(int argc, char **argv)
{

ros::init(argc, argv, "assignment2");
ros::NodeHandle nh;
ros::ServiceServer oracle = nh.advertiseService( "/oracle_hint", oracleCallback);
ros::ServiceServer service= nh.advertiseService("/oracle_solution", oracleService);
srand (time(NULL));


int uid;
for (int i = 0; i < 3; i++){	
	do{
		uid = rand()%6;
		for( int i = 0; i < 2; i++ ){
			if(uid == uIDs[i] ){
					uid = -1;
					break;
				}
			}
		}while(uid == -1);
		
	if(i==2){
		winID = uid;
	}
	else{
    uIDs[i] = uid;
	}
}

int c = 0;

for (int i = 0; i < 6; i++){
	if (i==uIDs[0] || i==uIDs[1] || i ==winID){
		oracle_msgs[c].ID=i;
		oracle_msgs[c].key="who";
		oracle_msgs[c].value=person[rand()%6];
		c++;
		oracle_msgs[c].ID=i;
		oracle_msgs[c].key="what";
		oracle_msgs[c].value=object[rand()%6];
		c++;
		oracle_msgs[c].ID=i;
		oracle_msgs[c].key="where";
		oracle_msgs[c].value=place[rand()%9];
		c++;
	}
	else{
		for (int j = 0; j < 5; j++){
			oracle_msgs[c].ID=i;
			oracle_msgs[c].key = key[rand()%3];
			if (oracle_msgs[c].key == "who")
				oracle_msgs[c].value = person[rand()%6];
			if (oracle_msgs[c].key == "what")
				oracle_msgs[c].value = object[rand()%6];
			if (oracle_msgs[c].key == "where")
				oracle_msgs[c].value = place[rand()%9];
			c++;
		}
	}
}

for (int i = 0; i < 6; i++){
	oracle_msgs[c].ID = rand() % 6;
	int a = rand()%5;
	if(a==0){
		oracle_msgs[c].key = "";
		oracle_msgs[c].value = "";
	}
	if (a==1){
		oracle_msgs[c].key="";
		oracle_msgs[c].value=person[rand()%6];
	}
	if (a==2){
		oracle_msgs[c].key="";
		oracle_msgs[c].value=object[rand()%6];
	}
	if (a==3){
		oracle_msgs[c].key="when";
		oracle_msgs[c].value="-1";
	}
	if (a==4){
		oracle_msgs[c].key="who";
		oracle_msgs[c].value="-1";
	}
	c++;
}

std::random_shuffle(std::begin(oracle_msgs), std::end(oracle_msgs));

ros::spin();

ros::shutdown();

return 0;
}
