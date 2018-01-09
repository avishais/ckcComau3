#include "apc_class.h"

// Constructor for the robots
two_robots::two_robots() {
	b = 344; // Height of base
	l1x = 524.5; 
	l1z = 199;
	l2 = 1250; 
	l3x = 186.5; 
	l3z = 210; 
	l4 = 1250; 
	l5 = 116.5; 
	lee = 22; 

	initMatrix(Q12, 4, 4);
	initMatrix(Q13, 4, 4);
	Q12 = {{cos(PI/3),-sin(PI/3),0,RD*cos(PI/3)+RD},{sin(PI/3),cos(PI/3),0,RD*sin(PI/3)},{0,0,1,0},{0,0,0,1}};
	//Q12 = {{1,0,0,800},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	Q13 = {{cos(-PI/3),-sin(-PI/3),0,RD*cos(-PI/3)+RD},{sin(-PI/3),cos(-PI/3),0,RD*sin(-PI/3)},{0,0,1,0},{0,0,0,1}};

	// Joint limits
	q1minmax = deg2rad(180);
	q2max = deg2rad(125);
	q2min = deg2rad(-60);
	q3max = deg2rad(75);
	q3min = deg2rad(-90);
	q4minmax = deg2rad(2700);
	q5minmax = deg2rad(120);
	q6minmax = deg2rad(400);

	// V_pose_rob_i_o is a vector of {x,y,z,angleZ} which is the position and orientation around the z axis of robot i
	V_pose_rob_1_o = {ROBOT1_X, ROBOT1_Y, ROBOT1_Z, ROBOT1_ROT};
	V_pose_rob_2_o = {ROBOT2_X, ROBOT2_Y, ROBOT2_Z, ROBOT2_ROT};
	V_pose_rob_3_o = {ROBOT3_X, ROBOT3_Y, ROBOT3_Z, ROBOT3_ROT};

	initMatrix(T_fk_solution_1, 4, 4);
	initVector(p_fk_solution_1, 3);
	initMatrix(T_fk_solution_2, 4, 4);
	initVector(p_fk_solution_2, 3);
	initMatrix(T_fk_solution_3, 4, 4);
	initVector(p_fk_solution_3, 3);
	initMatrix(T_fk_temp, 4, 4);
	initVector(p_fk_temp, 3);
	initVector(V_pose, 4);
	initMatrix(T1, 4, 4);
	initMatrix(T2, 4, 4);
	initMatrix(T3, 4, 4);
	initMatrix(T_mult_temp,4, 4);
	initMatrix(Q_IK_solutions_1, NUM_IK_SOLUTIONS, 6);
	initMatrix(Q_IK_solutions_2, NUM_IK_SOLUTIONS, 6);
	initMatrix(Q_IK_solutions_3, NUM_IK_SOLUTIONS, 6);
	initVector(valid_IK_solutions_indices_1, NUM_IK_SOLUTIONS);
	initVector(valid_IK_solutions_indices_2, NUM_IK_SOLUTIONS);
	initVector(valid_IK_solutions_indices_3, NUM_IK_SOLUTIONS);
	
	cout << "Robots initialized." << endl;
}

// -----FK-------

// Solves the FK of one robot
void two_robots::FKsolve_rob(State q, int robot_num) {

	switch (robot_num) {
		case 1: V_pose = V_pose_rob_1_o; break;
		case 2: V_pose = V_pose_rob_2_o; break;
		case 3: V_pose = V_pose_rob_3_o; break;
	}

	T_fk_temp[0][0] = -cos(V_pose[3])*(sin(q[4])*(sin(q[0])*sin(q[3])+cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))-cos(q[4])*(cos(q[0])*cos(q[1])*cos(q[2])-cos(q[0])*sin(q[1])*sin(q[2])))-sin(V_pose[3])*(sin(q[4])*(cos(q[0])*sin(q[3])-cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))-cos(q[4])*(sin(q[0])*sin(q[1])*sin(q[2])-cos(q[1])*cos(q[2])*sin(q[0])));
	T_fk_temp[0][1] = -sin(V_pose[3])*(cos(q[5])*(cos(q[0])*cos(q[3])+sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))-sin(q[5])*(cos(q[4])*(cos(q[0])*sin(q[3])-cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))+sin(q[4])*(sin(q[0])*sin(q[1])*sin(q[2])-cos(q[1])*cos(q[2])*sin(q[0]))))-cos(V_pose[3])*(cos(q[5])*(cos(q[3])*sin(q[0])-sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))-sin(q[5])*(cos(q[4])*(sin(q[0])*sin(q[3])+cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))+sin(q[4])*(cos(q[0])*cos(q[1])*cos(q[2])-cos(q[0])*sin(q[1])*sin(q[2]))));
	T_fk_temp[0][2] = sin(V_pose[3])*(sin(q[5])*(cos(q[0])*cos(q[3])+sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))+cos(q[5])*(cos(q[4])*(cos(q[0])*sin(q[3])-cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))+sin(q[4])*(sin(q[0])*sin(q[1])*sin(q[2])-cos(q[1])*cos(q[2])*sin(q[0]))))+cos(V_pose[3])*(sin(q[5])*(cos(q[3])*sin(q[0])-sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))+cos(q[5])*(cos(q[4])*(sin(q[0])*sin(q[3])+cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))+sin(q[4])*(cos(q[0])*cos(q[1])*cos(q[2])-cos(q[0])*sin(q[1])*sin(q[2]))));
	T_fk_temp[0][3] = V_pose[0]-sin(V_pose[3])*(l1x*sin(q[0])+l4*cos(q[1]+q[2])*sin(q[0])+l3x*cos(q[1]+q[2])*sin(q[0])+l3z*sin(q[1]+q[2])*sin(q[0])+l2*sin(q[0])*sin(q[1])+l5*cos(q[1]+q[2])*cos(q[4])*sin(q[0])+lee*cos(q[1]+q[2])*cos(q[4])*sin(q[0])+l5*cos(q[0])*sin(q[3])*sin(q[4])+lee*cos(q[0])*sin(q[3])*sin(q[4])-l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4])-l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])-lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4])-lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]))+cos(V_pose[3])*(l1x*cos(q[0])+l4*cos(q[1]+q[2])*cos(q[0])+l3x*cos(q[1]+q[2])*cos(q[0])+l3z*sin(q[1]+q[2])*cos(q[0])+l2*cos(q[0])*sin(q[1])+l5*cos(q[1]+q[2])*cos(q[0])*cos(q[4])+lee*cos(q[1]+q[2])*cos(q[0])*cos(q[4])-l5*sin(q[0])*sin(q[3])*sin(q[4])-lee*sin(q[0])*sin(q[3])*sin(q[4])-l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4])-l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])-lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4])-lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]));
	T_fk_temp[1][0] = -sin(V_pose[3])*(sin(q[4])*(sin(q[0])*sin(q[3])+cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))-cos(q[4])*(cos(q[0])*cos(q[1])*cos(q[2])-cos(q[0])*sin(q[1])*sin(q[2])))+cos(V_pose[3])*(sin(q[4])*(cos(q[0])*sin(q[3])-cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))-cos(q[4])*(sin(q[0])*sin(q[1])*sin(q[2])-cos(q[1])*cos(q[2])*sin(q[0])));
	T_fk_temp[1][1] = cos(V_pose[3])*(cos(q[5])*(cos(q[0])*cos(q[3])+sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))-sin(q[5])*(cos(q[4])*(cos(q[0])*sin(q[3])-cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))+sin(q[4])*(sin(q[0])*sin(q[1])*sin(q[2])-cos(q[1])*cos(q[2])*sin(q[0]))))-sin(V_pose[3])*(cos(q[5])*(cos(q[3])*sin(q[0])-sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))-sin(q[5])*(cos(q[4])*(sin(q[0])*sin(q[3])+cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))+sin(q[4])*(cos(q[0])*cos(q[1])*cos(q[2])-cos(q[0])*sin(q[1])*sin(q[2]))));
	T_fk_temp[1][2] = -cos(V_pose[3])*(sin(q[5])*(cos(q[0])*cos(q[3])+sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))+cos(q[5])*(cos(q[4])*(cos(q[0])*sin(q[3])-cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2])+cos(q[2])*sin(q[0])*sin(q[1])))+sin(q[4])*(sin(q[0])*sin(q[1])*sin(q[2])-cos(q[1])*cos(q[2])*sin(q[0]))))+sin(V_pose[3])*(sin(q[5])*(cos(q[3])*sin(q[0])-sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))+cos(q[5])*(cos(q[4])*(sin(q[0])*sin(q[3])+cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2])+cos(q[0])*cos(q[2])*sin(q[1])))+sin(q[4])*(cos(q[0])*cos(q[1])*cos(q[2])-cos(q[0])*sin(q[1])*sin(q[2]))));
	T_fk_temp[1][3] = V_pose[1]+cos(V_pose[3])*(l1x*sin(q[0])+l4*cos(q[1]+q[2])*sin(q[0])+l3x*cos(q[1]+q[2])*sin(q[0])+l3z*sin(q[1]+q[2])*sin(q[0])+l2*sin(q[0])*sin(q[1])+l5*cos(q[1]+q[2])*cos(q[4])*sin(q[0])+lee*cos(q[1]+q[2])*cos(q[4])*sin(q[0])+l5*cos(q[0])*sin(q[3])*sin(q[4])+lee*cos(q[0])*sin(q[3])*sin(q[4])-l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4])-l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])-lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4])-lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]))+sin(V_pose[3])*(l1x*cos(q[0])+l4*cos(q[1]+q[2])*cos(q[0])+l3x*cos(q[1]+q[2])*cos(q[0])+l3z*sin(q[1]+q[2])*cos(q[0])+l2*cos(q[0])*sin(q[1])+l5*cos(q[1]+q[2])*cos(q[0])*cos(q[4])+lee*cos(q[1]+q[2])*cos(q[0])*cos(q[4])-l5*sin(q[0])*sin(q[3])*sin(q[4])-lee*sin(q[0])*sin(q[3])*sin(q[4])-l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4])-l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])-lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4])-lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]));
	T_fk_temp[2][0] = -sin(q[1]+q[2])*cos(q[4])-cos(q[1]+q[2])*cos(q[3])*sin(q[4]);
	T_fk_temp[2][1] = -sin(q[5])*(sin(q[1]+q[2])*sin(q[4])-cos(q[1]+q[2])*cos(q[3])*cos(q[4]))+cos(q[1]+q[2])*cos(q[5])*sin(q[3]);
	T_fk_temp[2][2] = -cos(q[5])*(sin(q[1]+q[2])*sin(q[4])-cos(q[1]+q[2])*cos(q[3])*cos(q[4]))-cos(q[1]+q[2])*sin(q[3])*sin(q[5]);
	T_fk_temp[2][3] = b+l1z+V_pose[2]+l3z*cos(q[1]+q[2])-l4*sin(q[1]+q[2])-l3x*sin(q[1]+q[2])+l2*cos(q[1])-l5*cos(q[1]+q[2])*sin(q[3]+q[4])*(1.0/2.0)-lee*cos(q[1]+q[2])*sin(q[3]+q[4])*(1.0/2.0)-l5*sin(q[1]+q[2])*cos(q[4])-lee*sin(q[1]+q[2])*cos(q[4])+l5*sin(q[3]-q[4])*cos(q[1]+q[2])*(1.0/2.0)+lee*sin(q[3]-q[4])*cos(q[1]+q[2])*(1.0/2.0);
	T_fk_temp[3][0] = 0;
	T_fk_temp[3][1] = 0;
	T_fk_temp[3][2] = 0;
	T_fk_temp[3][3] = 1;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 4; j++)
			T_fk_temp[i][j] = fabs(T_fk_temp[i][j]) < 1e-4 ? 0 : T_fk_temp[i][j];

	// q - joint angles
	p_fk_temp = {T_fk_temp[0][3], T_fk_temp[1][3], T_fk_temp[2][3]};

	switch (robot_num) {
		case 1: p_fk_solution_1 = p_fk_temp; T_fk_solution_1 = T_fk_temp; break;
		case 2: p_fk_solution_2 = p_fk_temp; T_fk_solution_2 = T_fk_temp; break;
		case 3: p_fk_solution_3 = p_fk_temp; T_fk_solution_3 = T_fk_temp; break;
	}
}

// Get the FK solution (homogeneous matrix) of robot 1
Matrix two_robots::get_FK_solution_T1() {

	return T_fk_solution_1;
}

// Get the FK solution (only pose) of robot 1
State two_robots::get_FK_solution_p1() {

	return p_fk_solution_1;
}

// Get the FK solution (homogeneous matrix) of robot 2
Matrix two_robots::get_FK_solution_T2() {

	return T_fk_solution_2;
}

// Get the FK solution (only pose) of robot 2
State two_robots::get_FK_solution_p2() {

	return p_fk_solution_2;
}

// Get the FK solution (homogeneous matrix) of robot 3
Matrix two_robots::get_FK_solution_T3() {

	return T_fk_solution_3;
}

// Get the FK solution (only pose) of robot 3
State two_robots::get_FK_solution_p3() {

	return p_fk_solution_3;
}

int two_robots::get_countSolutions() {
	return countSolutions;
}

// -------IK----------

// Solves the IK problem of robot given the transformation matrix, robot number and required IK soluion index
bool two_robots::IKsolve_rob(Matrix T, int robot_num, int solution_num) {
	State q(6);

	double q1_add_sol[NUM_IK_SOLUTIONS] = { 0, 0, 0, 0, PI_, PI_, PI_, PI_};
	double q2_sign[NUM_IK_SOLUTIONS] = { -1, -1, -1, -1,  1,  1,  1,  1};
	double q3_sign[NUM_IK_SOLUTIONS] = { 1, -1,  1, -1,  1, -1,  1, -1};
	double sign456[NUM_IK_SOLUTIONS] = { 1, -1, -1,  1, -1,  1,  1, -1};
	State V_pose(3);

	switch (robot_num) {
		case 1: V_pose = V_pose_rob_1_o; break;
		case 2: V_pose = V_pose_rob_2_o; break;
		case 3: V_pose = V_pose_rob_3_o; break;
	}

	Matrix R = {{ cos(V_pose[3])*T[0][0] + sin(V_pose[3])*T[1][0], cos(V_pose[3])*T[0][1] + sin(V_pose[3])*T[1][1], cos(V_pose[3])*T[0][2] + sin(V_pose[3])*T[1][2] },
		{ cos(V_pose[3])*T[1][0] - sin(V_pose[3])*T[0][0], cos(V_pose[3])*T[1][1] - sin(V_pose[3])*T[0][1], cos(V_pose[3])*T[1][2] - sin(V_pose[3])*T[0][2]},
		{                                         T[2][0],                                         T[2][1],                                         T[2][2] }};
	 

	State p = { cos(V_pose[3])*T[0][3] + sin(V_pose[3])*T[1][3] - cos(V_pose[3])*T[3][3] * V_pose[0] - sin(V_pose[3])*T[3][3] * V_pose[1],
		cos(V_pose[3])*T[1][3] - sin(V_pose[3])*T[0][3] - cos(V_pose[3])*T[3][3] * V_pose[1] + sin(V_pose[3])*T[3][3] * V_pose[0],
		T[2][3] - V_pose[2]};

	State x6 = { R[0][0], R[1][0], R[2][0] };
	State p5 = { p[0] - x6[0] * (l5 + lee), p[1] - x6[1] * (l5 + lee), p[2] - x6[2] * (l5 + lee) };
	
	//q1
	{
		q[0] = atan2(p5[1], p5[0]) + q1_add_sol[solution_num];
		if (q[0]>PI_)
			q[0] -= 2*PI_;
		if (q[0]<-PI_)
			q[0] += 2*PI_;

		if (include_joint_limits && fabs(q[0]) > q1minmax)
			return false;

	}

	double k = sqrt(p5[0] * p5[0] + p5[1] * p5[1]); 
	double kv2 = (k + q2_sign[solution_num] * l1x) * (k + q2_sign[solution_num] * l1x);
	double D2 = kv2 + (p5[2] - (l1z + b)) * (p5[2] - (l1z + b)); // = D ^ 2
	double l34 = sqrt((l3z*l3z + (l3x + l4)*(l3x + l4))); // Distance from joint 23 axis to joint 45 axis.
	
	//q3
	double sinphi;
	{
		double alpha = atan2(l3x + l4, l3z); // Angle in the base of the triangle of link 3.
		double cosphi = (D2 - l2*l2 - l34*l34) / (2 * l2*l34);
		sinphi = (1 - cosphi*cosphi);
		//if (fabs(sinphi) < 1e-4) // In case sampling near singularity
			//sinphi = fabs(sinphi);
		if (sinphi < 0) {
			return false;
		}
		sinphi = q3_sign[solution_num]*sqrt(sinphi);
		q[2] = -(atan2(sinphi, cosphi)+alpha);
		if (q[2]>PI_)
			q[2] -= 2*PI_;
		if (q[2]<-PI_)
			q[2] += 2*PI_;
		if (include_joint_limits && (q[2] < q3min || q[2] > q3max))
			return false;
	}

	//q2
	{
		double sinalpha1 = -l34*(sinphi) / sqrt(D2);
		double alpha1 = atan2(-q2_sign[solution_num]*sinalpha1, sqrt(1 - sinalpha1*sinalpha1));
		double alpha2 = atan2(p5[2] - l1z - b, sqrt(kv2));
		q[1] = q2_sign[solution_num] * (alpha1 + alpha2 - PI_ / 2);
		if (q[1]>PI_)
			q[1] -= 2*PI_;
		if (q[1]<-PI_)
			q[1] += 2*PI_;
		if (include_joint_limits && (q[1] < q2min || q[1] > q2max))
			return false;

	}

	// Check feasibility of the first three joints
	/*if (fabs(p5[0]-(cos(q[1] + q[2])*cos(q[0])*(l4 + l3b) + l3a*sin(q[1] + q[2])*cos(q[0]) + l2*cos(q[0])*sin(q[1]))) > 0.2 || \
			fabs(p5[1]-(cos(q[1] + q[2])*sin(q[0])*(l4 + l3b) + l3a*sin(q[1] + q[2])*sin(q[0]) + l2*sin(q[0])*sin(q[1])) > 0.2 || \
					fabs(p5[2]-(b + l1 - sin(q[1] + q[2])*(l4 + l3b) + l3a*cos(q[1] + q[2]) + l2*cos(q[1]))) > 0.2))
					return false;*/

	// Solved according to Spong's "Robot dynamics and control" page 91,96.
	double c23 = cos(q[1] + q[2]); 
	double s23 = sin(q[1] + q[2]);
	double c1 = cos(q[0]); 
	double s1 = sin(q[0]);

	//q5
	{
		double S = R[0][0]*c23*c1 - R[2][0]*s23 + R[1][0]*c23*s1;
		q[4] = atan2(sign456[solution_num]*sqrt(1 - S*S), S);
		q[4] = fabs(q[4]) < 1e-4 ? 0 : q[4];
		if (include_joint_limits && fabs(q[4]) > q5minmax)
			return false;

	}

	// q4 and a6
	if (q[4]) {
		//q4
		double sinq4 = R[1][0]*c1 - R[0][0]*s1;
		double cosq4 = R[2][0]*c23 + R[0][0]*s23*c1 + R[1][0]*s23*s1;
		q[3] = atan2(sign456[solution_num]*sinq4, -sign456[solution_num]*cosq4);

		//q6
		double sinq6 = R[0][1]*c23*c1 - R[2][1]*s23 + R[1][1]*c23*s1;
		double cosq6 = R[0][2]*c23*c1 - R[2][2]*s23 + R[1][2]*c23*s1;
		q[5] = atan2(sign456[solution_num]*sinq6, sign456[solution_num]*cosq6);
	}
	else {
		//q4
		q[3] = 0;
		//q6
		q[5] = atan2(R[2][1], R[2][2]); // or q6 = atan2(R(3, 2), R(2, 2)) or q6 = atan2(-R(2, 3), R(3, 3));
	}
	if (include_joint_limits && (fabs(q[3]) > q4minmax || fabs(q[5]) > q6minmax))
		return false;


	for (int i = 0; i < 6; i++)
		if (fabs(q[i])<1e-5)
			q[i] = 0;

	switch (robot_num) {
		case 1: q_IK_solution_1 = q; break;
		case 2: q_IK_solution_2 = q; break;
		case 3: q_IK_solution_3 = q; break;
	}
		
	return true;
}

// Computes all the IK solutions to 'Q_IK_solutions_1' matrix.
// Returns the number of existing solutions
int two_robots::calc_all_IK_solutions_1(Matrix T) {

	int countSolutions = 0;
	for (int i = 0; i < NUM_IK_SOLUTIONS; i++) {
		if (IKsolve_rob(T, 1, i)) {
			Q_IK_solutions_1[countSolutions] = q_IK_solution_1;
			valid_IK_solutions_indices_1[countSolutions] = i;
			countSolutions++;
		}
	}

	return countSolutions;
}

// Computes all the IK solutions to 'Q_IK_solutions_2' matrix.
// Returns the number of existing solutions
int two_robots::calc_all_IK_solutions_2(Matrix T) {

	int countSolutions = 0;
	for (int i = 0; i < NUM_IK_SOLUTIONS; i++) {
		if (IKsolve_rob(T, 2, i)) {
			Q_IK_solutions_2[countSolutions] = q_IK_solution_2;
			valid_IK_solutions_indices_2[countSolutions] = i;
			countSolutions++;
		}
	}

	return countSolutions;
}

// Computes all the IK solutions to 'Q_IK_solutions_2' matrix.
// Returns the number of existing solutions
int two_robots::calc_all_IK_solutions_3(Matrix T) {

	int countSolutions = 0;
	for (int i = 0; i < NUM_IK_SOLUTIONS; i++) {
		if (IKsolve_rob(T, 3, i)) {
			Q_IK_solutions_3[countSolutions] = q_IK_solution_3;
			valid_IK_solutions_indices_3[countSolutions] = i;
			countSolutions++;
		}
	}

	return countSolutions;
}

State two_robots::get_all_IK_solutions_1(int num_sol) {
	return Q_IK_solutions_1[num_sol];
}

State two_robots::get_all_IK_solutions_2(int num_sol) {
	return Q_IK_solutions_2[num_sol];
}

State two_robots::get_all_IK_solutions_3(int num_sol) {
	return Q_IK_solutions_3[num_sol];
}

State two_robots::get_IK_solution_q1() {
	return q_IK_solution_1;
}

State two_robots::get_IK_solution_q2() {
	return q_IK_solution_2;
}

State two_robots::get_IK_solution_q3() {
	return q_IK_solution_3;
}

int two_robots::get_valid_IK_solutions_indices_1(int i) {
	return valid_IK_solutions_indices_1[i];
}
int two_robots::get_valid_IK_solutions_indices_2(int i) {
	return valid_IK_solutions_indices_2[i];
}
int two_robots::get_valid_IK_solutions_indices_3(int i) {
	return valid_IK_solutions_indices_3[i];
}

// -----------------------

bool two_robots::check_angle_limits(State q) {
	// Returns true if all angles are within the joint limits

	if (fabs(q[0]) > q1minmax)
		return false;
	if (q[1] < q2min || q[1] > q2max)
		return false;
	if (q[2] < q3min || q[2] > q3max)
		return false;
	if (fabs(q[3]) > q4minmax)
		return false;
	if (fabs(q[4]) > q5minmax)
		return false;
	if (fabs(q[5]) > q6minmax)
		return false;

	if (fabs(q[6]) > q1minmax)
		return false;
	if (q[7] < q2min || q[7] > q2max)
		return false;
	if (q[8] < q3min || q[8] > q3max)
		return false;
	if (fabs(q[9]) > q4minmax)
		return false;
	if (fabs(q[10]) > q5minmax)
		return false;
	if (fabs(q[11]) > q6minmax)
		return false;

	return true;
}

//------------------------

double two_robots::normDistance(State q_a, State q_b) {

	double max = 0, cur;
	for (int i=0; i<q_a.size(); i++) {
		cur = fabs(q_a[i]-q_b[i]);
		if (cur > max)
			max = cur;
	}
	return max;

	/*double sum = 0;
	for (int i=0; i<q_a.size(); i++)
		sum += pow(q_a[i]-q_b[i],2);
	return sum;*/
}

// Compute a specific IK solution when q1 is the active chain
// Checks that IK exists for Robot 2 & 3
bool two_robots::calc_specific_IK_solution_R1(State q1, int IKsol2, int IKsol3) {
	// q1 - angles of robot 1.

	FKsolve_rob(q1, 1);

	T2 = MatricesMult(get_FK_solution_T1(), Q12); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2

	if (!IKsolve_rob(T2, 2, IKsol2))
		return false;

	T3 = MatricesMult(get_FK_solution_T1(), Q13); // Returns the opposing required matrix of the rods tip at robot 2
	T3 = MatricesMult(T3, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2

	if (IKsolve_rob(T3, 3, IKsol3))
		return true;

	return false;
}

// Compute a specific IK solution when q2 is the active chain
// Checks that IK exists for Robot 1 & 3
bool two_robots::calc_specific_IK_solution_R2(State q2, int IKsol1, int IKsol3) {
	// q2 - angles of robot 2.

	Matrix Tinv = Q12;
	InvertMatrix(Q12, Tinv); // Invert matrix

	FKsolve_rob(q2, 2);
	T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	
	if (!IKsolve_rob(T1, 1, IKsol1))
		return false;

	T3 = MatricesMult(T1, Q13); // Returns the REQUIRED matrix of the rods tip at rob
	T3 = MatricesMult(T3, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 3

	if (IKsolve_rob(T3, 3, IKsol3))
		return true;

	return false;
}

// Compute a specific IK solution when q3 is the active chain
// Checks that IK exists for Robot 1 & 2
bool two_robots::calc_specific_IK_solution_R3(State q3, int IKsol1, int IKsol2) {
	// q3 - angles of robot 3.

	Matrix Tinv = Q13;
	InvertMatrix(Q13, Tinv); // Invert matrix

	FKsolve_rob(q3, 3);
	T1 = MatricesMult(get_FK_solution_T3(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob

	if (!IKsolve_rob(T1, 1, IKsol1))
		return false;

	T2 = MatricesMult(T1, Q12); // Returns the REQUIRED matrix of the rods tip at rob
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2	

	if (IKsolve_rob(T2, 2, IKsol2))
		return true;

	return false;
}

// =-------------------------------------------------------------=

// Matrix multiplication
Matrix two_robots::MatricesMult(Matrix M1, Matrix M2) {
	clearMatrix(T_mult_temp);
	for(unsigned i = 0; i < 4; ++i)
		for(unsigned j = 0; j < 4; ++j)
			for(int k = 0; k < 4; ++k)
			{
				T_mult_temp[i][j] += M1[i][k] * M2[k][j];
			}
	return T_mult_temp;
}

// 4x4 matrix invertion
bool two_robots::InvertMatrix(Matrix M, Matrix &Minv) {
	State m(16),  inv(16);

	int k = 0;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++, k++)
			m[k] = M[j][i];

    double det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] -
             m[5]  * m[11] * m[14] -
             m[9]  * m[6]  * m[15] +
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] -
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
              m[4]  * m[11] * m[14] +
              m[8]  * m[6]  * m[15] -
              m[8]  * m[7]  * m[14] -
              m[12] * m[6]  * m[11] +
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
             m[4]  * m[11] * m[13] -
             m[8]  * m[5] * m[15] +
             m[8]  * m[7] * m[13] +
             m[12] * m[5] * m[11] -
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] -
               m[8]  * m[6] * m[13] -
               m[12] * m[5] * m[10] +
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
              m[1]  * m[11] * m[14] +
              m[9]  * m[2] * m[15] -
              m[9]  * m[3] * m[14] -
              m[13] * m[2] * m[11] +
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
             m[0]  * m[11] * m[14] -
             m[8]  * m[2] * m[15] +
             m[8]  * m[3] * m[14] +
             m[12] * m[2] * m[11] -
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
              m[0]  * m[11] * m[13] +
              m[8]  * m[1] * m[15] -
              m[8]  * m[3] * m[13] -
              m[12] * m[1] * m[11] +
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
              m[0]  * m[10] * m[13] -
              m[8]  * m[1] * m[14] +
              m[8]  * m[2] * m[13] +
              m[12] * m[1] * m[10] -
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
             m[1]  * m[7] * m[14] -
             m[5]  * m[2] * m[15] +
             m[5]  * m[3] * m[14] +
             m[13] * m[2] * m[7] -
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
              m[0]  * m[7] * m[14] +
              m[4]  * m[2] * m[15] -
              m[4]  * m[3] * m[14] -
              m[12] * m[2] * m[7] +
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
              m[0]  * m[7] * m[13] -
              m[4]  * m[1] * m[15] +
              m[4]  * m[3] * m[13] +
              m[12] * m[1] * m[7] -
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
               m[0]  * m[6] * m[13] +
               m[4]  * m[1] * m[14] -
               m[4]  * m[2] * m[13] -
               m[12] * m[1] * m[6] +
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
              m[1] * m[7] * m[10] +
              m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] -
              m[9] * m[2] * m[7] +
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
             m[0] * m[7] * m[10] -
             m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] -
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
               m[0] * m[7] * m[9] +
               m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] -
               m[8] * m[1] * m[7] +
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
              m[0] * m[6] * m[9] -
              m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] +
              m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    k = 0;
    for (int i = 0; i < 4; i++)
    	for (int j = 0; j < 4; j++, k++)
    		Minv[j][i] = inv[k] * det;

    return true;
}

//--------- Misc ---------

// Initialize Matrix
void two_robots::initMatrix(Matrix &M, int n, int m) {
	M.resize(n);
	for (int i = 0; i < n; ++i)
		M[i].resize(m);
}

// Initialize State
void two_robots::initVector(State &V, int n) {
	V.resize(n);
}

// Convert degrees to radians
double two_robots::deg2rad(double deg) {
	return deg * PI_ / 180.0;
}

// Print matrix data to console
void two_robots::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

// Print vector data to console
void two_robots::printVector(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}

// Reset matrix data to zero
void two_robots::clearMatrix(Matrix &M) {
	for (unsigned i=0; i<M.size(); ++i)
		for (unsigned j=0; j<M[i].size(); ++j)
			M[i][j] = 0;;
}

int two_robots::get_IK_number() {
	return IK_number;
}

Matrix two_robots::get_T2() {
	return T2;
}

void two_robots::log_q(State q) {
	std::ofstream myfile;
	myfile.open("../paths/path.txt");

	myfile << 1 << endl;

	for (int i = 0; i < q.size(); i++)
		myfile << q[i] << " ";
	myfile << endl;

	myfile.close();
}


// ==================================
