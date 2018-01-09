#pragma once
/* This code defines the kinematics (IK and FK) of two COMAU NJ 16 robotic arms */

#include <iostream>
#include <vector>
#include <fstream>
#include <ctime>
#include <math.h>

#include "def.h"

#define PI_ 3.1416
#define NUM_IK_SOLUTIONS 8

using namespace std;

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class two_robots
{
private:
	double b, l1x, l1z, l2, l3x, l3z, l4, l5, l6, lee; // Links lengths
	double q1minmax, q2min, q2max, q3min, q3max, q4minmax, q5minmax, q6minmax; // Joint limits
	double L; // Length of grasped rod
	State V_pose_rob_1_o; // The vector [x,y,z,theta] that describes the position and orientation of robot 1 relative to origin (assume on the same x-y plane)
	State V_pose_rob_2_o; // The vector [x,y,z,theta] that describes the position and orientation of robot 2 relative to origin (assume on the same x-y plane)
	State V_pose_rob_3_o; // The vector [x,y,z,theta] that describes the position and orientation of robot 3 relative to origin (assume on the same x-y plane)	
	// FK parameters
	Matrix T_fk_solution_1; // FK solution of robot 1
	State p_fk_solution_1; // FK solution of robot 1
	Matrix T_fk_solution_2; // FK solution of robot 2
	State p_fk_solution_2; // FK solution of robot 2
	Matrix T_fk_solution_3; // FK solution of robot 3
	State p_fk_solution_3; // FK solution of robot 3
	// IK parameters
	State q_IK_solution_1; // IK solution of robot 1
	State q_IK_solution_2; // IK solution of robot 2
	State q_IK_solution_3; // IK solution of robot 3	
	Matrix Q_IK_solutions_1; // All IK solutions of robot 1
	Matrix Q_IK_solutions_2; // All IK solutions of robot 2
	Matrix Q_IK_solutions_3; // All IK solutions of robot 3	
	State valid_IK_solutions_indices_1; // Indices of the IK solutions to use in the IK_solve function for robot 1
	State valid_IK_solutions_indices_2; // Indices of the IK solutions to use in the IK_solve function for robot 2
	State valid_IK_solutions_indices_3; // Indices of the IK solutions to use in the IK_solve function for robot 3	
	int IK_number;
	int countSolutions;

	// Temp variable for time efficiency
	Matrix T_fk_temp;
	State p_fk_temp;
	State V_pose;
	Matrix T1, T2, T3, T_mult_temp;

public:
	/** Constructor */
	two_robots();

	/** Forward kinematics */
	void FKsolve_rob(State, int);
	Matrix get_FK_solution_T1();
	State get_FK_solution_p1();
	Matrix get_FK_solution_T2();
	State get_FK_solution_p2();
	Matrix get_FK_solution_T3();
	State get_FK_solution_p3();

	/** Inverse kinematics */
	bool IKsolve_rob(Matrix, int, int);
	State get_IK_solution_q1();
	State get_IK_solution_q2();
	State get_IK_solution_q3();	
	int get_countSolutions();
	bool calc_specific_IK_solution_R1(State, int, int);
	bool calc_specific_IK_solution_R2(State, int, int);
	bool calc_specific_IK_solution_R3(State, int, int);

	/** Is robots configurations feasible? */
	int calc_all_IK_solutions_1(Matrix);
	int calc_all_IK_solutions_2(Matrix);
	int calc_all_IK_solutions_3(Matrix);	

	/** Check the angles limits of the ABB */
	bool check_angle_limits(State q);

	// Getters
	int get_IK_number();
	Matrix get_T2();
	State get_all_IK_solutions_1(int);
	State get_all_IK_solutions_2(int);
	State get_all_IK_solutions_3(int);
	int get_valid_IK_solutions_indices_1(int);
	int get_valid_IK_solutions_indices_2(int);
	int get_valid_IK_solutions_indices_3(int);
	
	// Misc
	void initVector(State &, int);
	void initMatrix(Matrix &, int, int);
	double deg2rad(double);
	void printMatrix(Matrix);
	void printVector(State);
	Matrix MatricesMult(Matrix, Matrix);
	bool InvertMatrix(Matrix M, Matrix &Minv); // Inverse of a 4x4 matrix
	void clearMatrix(Matrix &);
	double normDistance(State, State);
	void log_q(State);

	Matrix Q12;
	Matrix Q13;
	Matrix getQ12() {
		return Q12;
	}
	Matrix getQ13() {
		return Q13;
	}


	State get_robots_properties() {
		State E = {b, l1x, l1z, l2, l3x, l3z, l4, l5, lee};
		return E;
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
