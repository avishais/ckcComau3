/*
 * kdl_class.h
 *
 *  Created on: Feb 27, 2017
 *      Author: avishai
 */

#ifndef KDL_CLASS_H_
#define KDL_CLASS_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolverpos_nr.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/tree.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>

// #include "MYtreeiksolverpos_nr_jl.hpp"
// #include "MYtreeiksolvervel_wdls.hpp"

#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "def.h"

#define PI 3.1416

using namespace std;
using namespace KDL;

typedef vector<double> State;
typedef vector<vector< double >> MAtrix;

class kdl
{
private:
	double b, l1x, l1z, l2, l3x, l3z, l4, l5, l6, lee; // Links lengths
	double q1minmax, q2min, q2max, q3min, q3max, q4minmax, q5minmax, q6minmax; // Joint limits

	// IK parameters
	State q_solution; // Full solution	

	// Temp variable for time efficiency
	MAtrix T_pose12, T_pose13, T_mult_temp;

	double L; // Rod length

	int numJoints;

public:
	// Constructor
	kdl();

	/** KDL declarations */
	KDL::Chain chain_rob2, chain_rob3, extract_chain12, extract_chain13;
	KDL::Tree tree;
	//ChainFkSolverPos_recursive fksolver;
	KDL::JntArray jointpositions, jointpositions1;
	KDL::Frame cartposFK; // Create the frame that will contain the FK results
	KDL::Frame cartposIK12, cartposIK13; // Create the frames that will contain the IK results
	KDL::JntArray q_min; // Minimum joint limits
	KDL::JntArray q_max; // Maximum joint limits

	/** Newton-Raphson projection onto the constraint surface */
	bool GD(State);
	State get_GD_result();
	
	/** Check the angles limits of the ABB - IK based on a constant trans. matrix */
	bool check_angle_limits(State);

	/**  Forward kinematics of the arm - only for validation */
	void FK(State, int);
	MAtrix get_FK_solution();
	MAtrix T_fk, T_fk_temp;

	/** Misc */
	void initVector(State &, int);
	void initMatrix(MAtrix &, int, int);
	double deg2rad(double);
	void printMatrix(MAtrix);
	void printVector(State);
	void clearMatrix(MAtrix &);
	MAtrix MatricesMult(MAtrix, MAtrix);		

	/** Log conf. to path.txt file */
	void log_q(State q);

	MAtrix Q12;
	MAtrix Q13;
	MAtrix getQ12() {
		return Q12;
	}
	MAtrix getQ13() {
		return Q13;
	}

	int get_numJoints() {
		return numJoints;
	}

	/** Returns ABB's link lengths */
	State get_robots_properties() {
		State P = {b, l1x, l1z, l2, l3x, l3z, l4, l5, lee};
		return P;
	}

	MAtrix get_Tpose12() {
		return T_pose12;
	}
	void set_Tpose12(MAtrix T) {
		T_pose12 = T;
	}
	MAtrix get_Tpose13() {
		return T_pose13;
	}
	void set_Tpose13(MAtrix T) {
		T_pose13 = T;
	}

	/** Performance parameters */
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}

	bool include_joint_limits = false;
};



#endif /* KDL_CLASS_H_ */
