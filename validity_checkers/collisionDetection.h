#ifndef COLLISIONDETECTION
#define COLLISIONDETECTION 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "PQP.h"
#include "model.h"
#include "/home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/simulator/MatVec.h"
#include <iostream>
#include <string>
#include <vector>

#include "../proj_classes/def.h"

typedef std::vector<std::vector< double > > Matrix;
typedef std::vector< double > State;

class collisionDetection
{
public:

	double offsetX, offsetY, offsetZ, offsetRot;
	collisionDetection();
	void load_models();
	int collision_state(State);
	int collision_state(State, State, State);
	PQP_Model base, link1, link2, link3, link4, link5, link6, EE, table;
	PQP_Model base2, link12, link22, link32, link42, link52, link62, EE2;
	PQP_Model base3, link13, link23, link33, link43, link53, link63, EE3;
	PQP_Model obs, obs1, obs2, wheel;

	// Performance parameters
	int collisionCheck_counter;
	double collisionCheck_time;
	int get_collisionCheck_counter() {
		return collisionCheck_counter;
	}
	double get_collisionCheck_time() {
		return collisionCheck_time;
	}

	int env;

	bool withObs = false;
};

#endif
