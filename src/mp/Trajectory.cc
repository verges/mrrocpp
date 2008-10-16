
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include "mp/Trajectory.h"

Trajectory::Trajectory()
{
	trjID = new char[80];
	trjPoses	= new std::list<ecp_smooth_taught_in_pose>();
}

Trajectory::Trajectory(char *numOfPoses, char *trajectoryID, char *poseSpecification)
{
	trjID = new char[80];
	strcpy(trjID, trajectoryID);
	this->numOfPoses = (uint64_t)atoi(numOfPoses);
	poseSpec = returnProperPS(poseSpecification);
	trjPoses	= new std::list<ecp_smooth_taught_in_pose>();
	
}

Trajectory::Trajectory(const Trajectory &trajectory)
{
	trjID = new char[80];
	strcpy(trjID, trajectory.trjID);
	numOfPoses = trajectory.numOfPoses;
	poseSpec = trajectory.poseSpec;
	trjPoses = new std::list<ecp_smooth_taught_in_pose>(trajectory.trjPoses->begin(), trajectory.trjPoses->end());
}

Trajectory::~Trajectory()
{
	delete []trjID;
	delete trjPoses;
}

void Trajectory::setTrjID(char *trjID)
{
	strcpy(this->trjID, trjID);
}

char * Trajectory::getTrjID() const
{
	return trjID;
}

POSE_SPECIFICATION Trajectory::returnProperPS(char *poseSpecification)
{
	if ( !strcmp(poseSpecification, (const char *)"MOTOR") )
	{	return MOTOR;	}
	if ( !strcmp(poseSpecification, (const char *)"JOINT") ) 
	{	return JOINT;	}
	if ( !strcmp(poseSpecification, (const char *)"XYZ_ANGLE_AXIS") ) 
	{	return XYZ_ANGLE_AXIS;	}
	if ( !strcmp(poseSpecification, (const char *)"XYZ_EULER_ZYZ") ) 
	{	return XYZ_EULER_ZYZ;	}
	else
		return INVALID_END_EFFECTOR;
}

int Trajectory::setValuesInArray(double arrayToFill[], char *dataString)
{
	int index = 0;
	char *value;
	char *toSplit = strdup(dataString);
	
	value = strtok(toSplit, " \t");
	arrayToFill[index++] = atof(value);
	while((value = strtok(NULL, " \t"))!=NULL)
		arrayToFill[index++] = atof(value);
	return index;
}

char * Trajectory::toString(double valArr[], int length)
{
	char * afterConv = new char[160];
	for(int i=0; i<length; i++)
	{
		if(i==0)
			sprintf(afterConv, "%g", valArr[i]);
		else
			sprintf(afterConv, "%s\t%g", afterConv, valArr[i]);
	}
//	std::cout<<afterConv<<std::endl;
	return afterConv;
}

char * Trajectory::toString(int numberOfPoses)
{
	char * numStr = new char[10];
	sprintf(numStr, "%d", numberOfPoses);

	return numStr;
}

char * Trajectory::returnRobotName(ROBOT_ENUM robot)
{

	if(robot == ROBOT_IRP6_ON_TRACK)
		return "ROBOT_IRP6_ON_TRACK";
	else if(robot == ROBOT_IRP6_POSTUMENT)
		return "ROBOT_IRP6_POSTUMENT";
	else if(robot == ROBOT_CONVEYOR)
		return "ROBOT_CONVEYOR";
	else if(robot == ROBOT_SPEAKER)
		return "ROBOT_SPEAKER";
	else if(robot == ROBOT_IRP6_MECHATRONIKA)
		return "ROBOT_IRP6_MECHATRONIKA";
	else if(robot == ROBOT_ELECTRON)
		return "ROBOT_ELECTRON";
	else if(robot == ROBOT_FESTIVAL)
		return "ROBOT_FESTIVAL";
	else if(robot == ROBOT_HAND)
		return "ROBOT_HAND";
	else if(robot == ROBOT_SPEECHRECOGNITION)
		return "ROBOT_SPEECHRECOGNITION";
	else
		return "ROBOT_UNDEFINED";	
}

char * Trajectory::toString(POSE_SPECIFICATION ps)
{
	if ( ps == MOTOR )
	{	return "MOTOR";	}
	if ( ps == JOINT ) 
	{	return "JOINT";	}
	if ( ps == XYZ_ANGLE_AXIS ) 
	{	return "XYZ_ANGLE_AXIS";	}
	if ( ps == XYZ_EULER_ZYZ ) 
	{	return "XYZ_EULER_ZYZ";	}
	else
		return "INVALID_END_EFFECTOR";
}

bool Trajectory::writeTrajectoryToXmlFile(char *fileName, POSE_SPECIFICATION ps, std::list<ecp_smooth_taught_in_pose> &poses)
{
	char * file = new char[80];
	int posCount = poses.size();
	xmlDocPtr doc;
	xmlNodePtr tree, subtree;
	std::list<ecp_smooth_taught_in_pose>::iterator it;
		
	doc = xmlNewDoc((const xmlChar *) "1.0");
	
	doc->children = xmlNewDocNode(doc, NULL, (const xmlChar *) "Trajectory", NULL);
	xmlSetProp(doc->children, (const xmlChar *) "coordinateType", (const xmlChar *) Trajectory::toString(ps));
	xmlSetProp(doc->children, (const xmlChar *) "numOfPoses", (const xmlChar *) Trajectory::toString(posCount));
	for(it = poses.begin(); it != poses.end(); ++it)
	{
		tree = xmlNewChild(doc->children, NULL, (const xmlChar *) "Pose", NULL);
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"StartVelocity", (const xmlChar *)Trajectory::toString((*it).v_p, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"EndVelocity", (const xmlChar *)Trajectory::toString((*it).v_k, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Velocity", (const xmlChar *)Trajectory::toString((*it).v, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Accelerations", (const xmlChar *)Trajectory::toString((*it).a, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Coordinates", (const xmlChar *)Trajectory::toString((*it).coordinates, MAX_SERVOS_NR));
	}
	sprintf(file, "%s%s", fileName, ".xml");

	xmlKeepBlanksDefault(0);
	xmlSaveFormatFile(file, doc, 1);
}

void Trajectory::createNewPose()
{
	actPose = new ecp_smooth_taught_in_pose();
	actPose->arm_type = this->poseSpec;
}

void Trajectory::addPoseToTrajectory()
{
	trjPoses->push_back(*actPose);
}

void Trajectory::setNumOfPoses(uint64_t numOfPoses)
{
	this->numOfPoses = numOfPoses;
}

uint64_t Trajectory::getNumberOfPoses() const
{
	return numOfPoses;
}

void Trajectory::setPoseSpecification(char *poseSpecification)
{
	poseSpec = returnProperPS(poseSpecification);
}

POSE_SPECIFICATION Trajectory::getPoseSpecification() const
{
	return poseSpec;
}

void Trajectory::setStartVelocities(char *startVelocities)
{
	setValuesInArray(actPose->v_p, startVelocities);
}

double * Trajectory::getStartVelocities() const
{
	return actPose->v_p;
}

void Trajectory::setEndVelocities(char *endVelocities)
{
	setValuesInArray(actPose->v_k, endVelocities);
}

double * Trajectory::getEndVelocities() const
{
	return actPose->v_k;
}

void Trajectory::setVelocities(char *Velocities)
{
	setValuesInArray(actPose->v, Velocities);
}

double * Trajectory::getVelocities() const
{
	return actPose->v;
}

void Trajectory::setAccelerations(char *accelerations)
{
	setValuesInArray(actPose->a, accelerations);
}

double * Trajectory::getAccelerations() const
{
	return actPose->a;
}

void Trajectory::setCoordinates(char *cCoordinates)
{
	setValuesInArray(actPose->coordinates, cCoordinates);
}

double * Trajectory::getCoordinates() const
{
	return actPose->coordinates;
}

void Trajectory::showTime()
{
	std::list<ecp_smooth_taught_in_pose>::iterator it;
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %d\n", trjID, poseSpec, numOfPoses);
	for(it=trjPoses->begin(); it!=trjPoses->end(); ++it)
	{
		printf("%f %f %f %f %f %f %f %f \n", (*it).v_p[0], (*it).v_p[1], (*it).v_p[2], (*it).v_p[3], 
				(*it).v_p[4], (*it).v_p[5], (*it).v_p[6], (*it).v_p[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).v_k[0], (*it).v_k[1], (*it).v_k[2], (*it).v_k[3], 
				(*it).v_k[4], (*it).v_k[5], (*it).v_k[6], (*it).v_k[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).v[0], (*it).v[1], (*it).v[2], (*it).v[3], 
				(*it).v[4], (*it).v[5], (*it).v[6], (*it).v[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).a[0], (*it).a[1], (*it).a[2], (*it).a[3], 
				(*it).a[4], (*it).a[5], (*it).a[6], (*it).a[7]);
		printf("%f %f %f %f %f %f %f %f \n***\n\n", (*it).coordinates[0], (*it).coordinates[1], (*it).coordinates[2], (*it).coordinates[3], 
				(*it).coordinates[4], (*it).coordinates[5], (*it).coordinates[6], (*it).coordinates[7]);
	}

}

std::list<ecp_smooth_taught_in_pose> * Trajectory::getPoses()
{
	return trjPoses;
}
	
