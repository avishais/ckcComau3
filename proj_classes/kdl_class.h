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
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "def.h"

#define PI 3.1416

using namespace std;
using namespace KDL;

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class kdl
{
private:
	double b, l1x, l1z, l2, l3x, l3z, l4, l5, l6, lee; // Links lengths
	double q1minmax, q2min, q2max, q3min, q3max, q4minmax, q5minmax, q6minmax; // Joint limits

	// IK parameters
	State q_solution12; // IK 12 solution
	State q_solution3; // IK 3 solution	
	State q_solution; // Full solution	

	// Temp variable for time efficiency
	Matrix T_pose12, T_pose3, T_pose1, T_mult_temp;

	double L; // Rod length

public:
	// Constructor
	kdl();

	/** KDL declarations */
	KDL::Chain chain, rob1, rob3;
	//ChainFkSolverPos_recursive fksolver;
	KDL::JntArray jointpositions, jointpositions1;
	KDL::Frame cartposFK; // Create the frame that will contain the FK results
	KDL::Frame cartposIK; // Create the frame that will contain the FK results

	/** Newton-Raphson projection onto the constraint surface */
	bool GD(State);
	bool GD12(State);
	State get_GD_result();
	State get_GD12_result();
	
	bool IK3(State, Matrix);
	State get_IK3_solution();		

	/** Check the angles limits of the ABB - IK based on a constant trans. matrix */
	bool check_angle_limits(State);

	/**  Forward kinematics of the arm - only for validation */
	void FK(State q);
	void FK13(State q, int);
	Matrix get_FK_solution();
	Matrix get_FK13_solution();
	Matrix T_fk, T_fk13, T_fk_temp;

	/** Misc */
	void initVector(State &, int);
	void initMatrix(Matrix &, int, int);
	double deg2rad(double);
	void printMatrix(Matrix);
	void printVector(State);
	void clearMatrix(Matrix &);
	Matrix MatricesMult(Matrix, Matrix);		

	/** Log conf. to path.txt file */
	void log_q(State q);

	Matrix Q12;
	Matrix Q13;
	Matrix getQ12() {
		return Q12;
	}
	Matrix getQ13() {
		return Q13;
	}

	/** Returns ABB's link lengths */
	State get_robots_properties() {
		State P = {b, l1x, l1z, l2, l3x, l3z, l4, l5, lee};
		return P;
	}

	Matrix get_Tpose12() {
		return T_pose12;
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

	bool include_joint_limits = true;
};



#endif /* KDL_CLASS_H_ */
