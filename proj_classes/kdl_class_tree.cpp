#include "kdl_class.h"

// Constructor for the robots
kdl::kdl() {
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

	initMatrix(T_fk, 4, 4);
	initMatrix(T_mult_temp,4, 4);	

	initMatrix(T_pose12, 4, 4);
	T_pose12 = {{cos(PI/3), -sin(PI/3), 0, ROBOT2_X-ROBOT1_X}, {sin(PI/3), cos(PI/3), 0, ROBOT2_Y-ROBOT1_Y}, {0, 0, 1, ROBOT2_Z-ROBOT1_Z}, {0, 0, 0, 1}};
	initMatrix(T_pose13, 4, 4);
	T_pose13 = {{cos(-PI/3), -sin(-PI/3), 0, ROBOT3_X-ROBOT1_X}, {sin(-PI/3), cos(-PI/3), 0, ROBOT3_Y-ROBOT1_Y}, {0, 0, 1, ROBOT3_Z-ROBOT1_Z}, {0, 0, 0, 1}};

	// cout << "T_pose13:" << endl;
	// printMatrix(T_pose13);

	string junction = "link1";

	//Definition of a kinematic chain & add segments to the chain
	// --- Robot 1
	tree.addSegment(Segment("base", Joint(Joint::None),Frame(Vector(0.0,0.0,b))), "root"); // Base link
	tree.addSegment(Segment("link1", Joint(Joint::RotZ),Frame(Vector(l1x,0.0,l1z))), "base"); // Link 1
	// tree.addSegment(Segment("link2", Joint(Joint::RotY),Frame(Vector(0.0,0.0,l2))), "link1"); // Link 2
	// tree.addSegment(Segment("link3", Joint(Joint::RotY),Frame(Vector(l3x,0.0,l3z))), "link2"); // Link 3
	// tree.addSegment(Segment("link4", Joint(Joint::RotX),Frame(Vector(l4,0.0,0.0))), "link3"); // Link 4
	// tree.addSegment(Segment("link5", Joint(Joint::RotY),Frame(Vector(l5,0.0,0.0))), "link4"); // Link 5
	// tree.addSegment(Segment("link6", Joint(Joint::RotX),Frame(Vector(lee,0.0,0.0))), "link5"); // EE

	// --- Robot 2 with wheel
	// Wheel
	Rotation r12( Q12[0][0],Q12[0][1],Q12[0][2], Q12[1][0],Q12[1][1],Q12[1][2], Q12[2][0],Q12[2][1],Q12[2][2] );
	chain_rob2.addSegment(Segment("wheel12t", Joint(Joint::None),Frame(Vector(Q12[0][3],Q12[1][3],Q12[2][3])))); // Box translation
	chain_rob2.addSegment(Segment("wheel12r", Joint(Joint::None),Frame(r12))); // Box rotation
	// Robot 2 - backwards
	chain_rob2.addSegment(Segment("link62", Joint(Joint::None),Frame(Vector(lee,0.0,0.0)))); // Box translation	
	chain_rob2.addSegment(Segment("link52", Joint(Joint::RotX),Frame(Vector(l5,0.0,0.0)))); // Link 5
	chain_rob2.addSegment(Segment("link42", Joint(Joint::RotY),Frame(Vector(l4,0.0,0.0)))); // Link 4
	chain_rob2.addSegment(Segment("link32", Joint(Joint::RotX),Frame(Vector(l3x,0.0,-l3z)))); // Link 3 "-"
	chain_rob2.addSegment(Segment("link22", Joint(Joint::RotY),Frame(Vector(0.0,0.0,-l2)))); // Link 2
	chain_rob2.addSegment(Segment("link12", Joint(Joint::RotY),Frame(Vector(l1x,0.0,-l1z)))); // Link 1
	chain_rob2.addSegment(Segment("base2", Joint(Joint::RotZ),Frame(Vector(0.0,0.0,-b)))); // Base link

	// --- Robot 3 with wheel
	// Wheel
	Rotation r13( Q13[0][0],Q13[0][1],Q13[0][2], Q13[1][0],Q13[1][1],Q13[1][2], Q13[2][0],Q13[2][1],Q13[2][2] );
	chain_rob3.addSegment(Segment("wheel13t", Joint(Joint::None),Frame(Vector(Q13[0][3],Q13[1][3],Q13[2][3])))); // Box translation
	chain_rob3.addSegment(Segment("wheel13r", Joint(Joint::None),Frame(r13))); // Box rotation
	// Robot 3 - backwards
	chain_rob3.addSegment(Segment("link63", Joint(Joint::None),Frame(Vector(lee,0.0,0.0)))); // Box translation	
	chain_rob3.addSegment(Segment("link53", Joint(Joint::RotX),Frame(Vector(l5,0.0,0.0)))); // Link 5
	chain_rob3.addSegment(Segment("link43", Joint(Joint::RotY),Frame(Vector(l4,0.0,0.0)))); // Link 4
	chain_rob3.addSegment(Segment("link33", Joint(Joint::RotX),Frame(Vector(l3x,0.0,-l3z)))); // Link 3 "-"
	chain_rob3.addSegment(Segment("link23", Joint(Joint::RotY),Frame(Vector(0.0,0.0,-l2)))); // Link 2
	chain_rob3.addSegment(Segment("link13", Joint(Joint::RotY),Frame(Vector(l1x,0.0,-l1z)))); // Link 1
	chain_rob3.addSegment(Segment("base3", Joint(Joint::RotZ),Frame(Vector(0.0,0.0,-b)))); // Base link

	// Connect the two chains to the EE of robot 1 (already in tree)
	tree.addChain(chain_rob2, junction);
	tree.addChain(chain_rob3, junction);
	
	// Extracts two chains that start from base of robot 1 to bases of the two other robots, respectively.
	// This is only for FK (can be done between them separately)
	tree.getChain("root", "base2", extract_chain12);
	tree.getChain("root", "base3", extract_chain13);
	
	// Create joint array
	unsigned int nj = tree.getNrOfJoints();
	jointpositions = JntArray(nj);
	numJoints = nj;
	unsigned int nj1 = extract_chain12.getNrOfJoints();
	jointpositions1 = JntArray(nj1);

	// Joint limits for KDL_IK_JL
	q_min = JntArray(nj);
	q_max = JntArray(nj);
	State q_min_vec = {-q1minmax, q2min, q3min, -q4minmax, -q5minmax, -q6minmax, -q6minmax, -q5minmax, -q4minmax, q3min, q2min, -q1minmax, -q6minmax, -q5minmax, -q4minmax, q3min, q2min, -q1minmax};
	State q_max_vec = {q1minmax, q2max, q3max, q4minmax, q5minmax, q6minmax, q6minmax, q5minmax, q4minmax, q3max, q2max, q1minmax, q6minmax, q5minmax, q4minmax, q3max, q2max, q1minmax};
	for (int i = 0; i < nj; i++) {
		q_min(i) = -10000;//q_min_vec[i];
		q_max(i) = 10000;//q_max_vec[i];
	}

	initVector(q_solution, nj);

	cout << "Initiated kinematic tree with " << numJoints << " joints.\n";
}


// ----- Descend -------

bool kdl::GD(State q_init_flip) {

	bool valid = true;
	int nj = q_init_flip.size();

	// Flip robot last two vectors corresponding to robots 2 & 3
	State q_init = q_init_flip;//(nj);
	// for (int i = 0; i < 6; i++)
	// 	q_init[i] = q_init_flip[i];
	// for (int i = 11, j = 6; i >= 6; i--,j++)
	// 	q_init[j] = q_init_flip[i];
	// q_init[11] = -q_init[11];
	// for (int i = 17, j = 12; i >= 12; i--,j++)
	// 	q_init[j] = q_init_flip[i];
	// q_init[17] = -q_init[17];

	State q(nj);

	IK_counter++;
	clock_t begin = clock();

	// Define goal position and orientation for both branches
	for (int i = 0; i < 3; i++) {
		cartposIK12.p(i) = T_pose12[i][3];
		cartposIK13.p(i) = T_pose13[i][3];
		for (int j = 0; j < 3; j++) {
			cartposIK12.M(i,j) = T_pose12[i][j];
			cartposIK13.M(i,j) = T_pose13[i][j];
		}
	}
	KDL::Frames F_dest;
    F_dest.insert(std::pair<std::string, Frame>("base2", cartposIK12));
    F_dest.insert(std::pair<std::string, Frame>("base3", cartposIK13));

	// KDL
	TreeFkSolverPos_recursive fksolver(tree);
    TreeIkSolverVel_wdls iksolverv(tree, {"base2","base3"}); //Inverse velocity solver
    TreeIkSolverPos_NR_JL iksolver(tree, {"base2","base3"}, q_min, q_max, fksolver, iksolverv,1000,1e-3);//Maximum 100 iterations, stop at accuracy 1e-6

	// return false;
	//Creation of jntarrays:
	JntArray qKDL(tree.getNrOfJoints());
	JntArray qInit(tree.getNrOfJoints());

	for (int i = 0; i < nj; i++)
		qInit(i) = q_init[i];

	// Solve
	double ret = iksolver.CartToJnt(qInit, F_dest, qKDL);
	cout << "** Final ret: " << ret << endl;

	// for (int i = 0; i < nj; i++)
	// 	cout << qKDL(i) << " ";
	// cout << endl;

	bool result = false;
	if (ret >= 0) {
		for (int i = 0; i < nj; i++)
			if (fabs(qKDL(i)) < 1e-4)
				q[i] = 0;
			else
				q[i] = qKDL(i);

		for (int i = 0; i < q.size(); i++) {
			q[i] = fmod(q[i], 2*PI);
			if (q[i]>PI)
				q[i] -= 2*PI;
			if (q[i]<-PI)
				q[i] += 2*PI;
		}

		// for (int i = 0; i < 6; i++)
		// 	q_solution[i] = q[i];
		// for (int i = 11, j = 6; i >= 6; i--,j++)
		// 	q_solution[j] = q[i];
		// q_solution[6] = -q_solution[6];
		// for (int i = 17, j = 12; i >= 12; i--,j++)
		// 	q_solution[j] = q[i];
		// q_solution[12] = -q_solution[12];
		q_solution = q;

		if (include_joint_limits && !check_angle_limits(q_solution))
			result = false;
		else
			result = true;
	}

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return result;
}

State kdl::get_GD_result() {
	return q_solution;
}


bool kdl::check_angle_limits(State q) {

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

	if (q.size()==6)
		return true;

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



// -----FK-------

// This is only for validation. There is no use for this function in terms of closed chain kinematics
void kdl::FK(State qfull, int chain_num) {
	// chain_num = 1, robots 1 & 2
	// chain_num = 2, robots 1 & 3

	int n = chain_num==1 ? extract_chain12.getNrOfJoints() : extract_chain13.getNrOfJoints();

	// Take only the angles of the two robots in question
	State q(n);
	q[0] = qfull[0];
	if (chain_num == 1) {
		for (int i = 1; i < 7; i++)
			q[i] = qfull[i];
	}
	else
		for (int i = 7, j = 0; i < 13; i++,j++)
			q[j] = qfull[i];

	// for (int i = 0; i < 6; i++)
	// 	q[i] = qfull[i];
	// int st = chain_num==1 ? 6 : 12;
	// for (int i = st, j = 6; i < st+6; i++, j++)
	// 	q[j] = qfull[i];

	// // Flip robot two vector
	// State q_flip(q.size());
	// for (int i = 0; i < 6; i++)
	// 	q_flip[i] = q[i];
	// for (int i = 11, j = 6; i >= 6; i--,j++)
	// 	q_flip[j] = q[i];
	// q_flip[11] = -q_flip[11];
	State q_flip = q;

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = chain_num==1 ? ChainFkSolverPos_recursive(extract_chain12) : ChainFkSolverPos_recursive(extract_chain13);

	for (int i = 0; i < n; i++)
		jointpositions1(i) = q_flip[i];

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions1, cartposFK);

	for (int i = 0; i < 3; i++) {
		T_fk[i][3] = cartposFK.p(i);
		for (int j = 0; j < 3; j++) {
			T_fk[i][j] = cartposFK.M(i,j);
			T_fk[i][j] = fabs(T_fk[i][j]) < 1e-4 ? 0 : T_fk[i][j];
		}
	}
	T_fk[3][0] = T_fk[3][1] = T_fk[3][2] = 0;
	T_fk[3][3] = 1;
}

MAtrix kdl::get_FK_solution() {

	return T_fk;
}

//------------------------

// Misc
void kdl::initMatrix(MAtrix &M, int n, int m) {
	M.resize(n);
	for (int i = 0; i < n; ++i)
		M[i].resize(m);
}

void kdl::initVector(State &V, int n) {
	V.resize(n);
}

double kdl::deg2rad(double deg) {
	return deg * PI / 180.0;
}

void kdl::printMatrix(MAtrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void kdl::printVector(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}


void kdl::clearMatrix(MAtrix &M) {
	for (unsigned i=0; i<M.size(); ++i)
		for (unsigned j=0; j<M[i].size(); ++j)
			M[i][j] = 0;;
}

// Matrix multiplication
MAtrix kdl::MatricesMult(MAtrix M1, MAtrix M2) {
	clearMatrix(T_mult_temp);
	for(unsigned i = 0; i < 4; ++i)
		for(unsigned j = 0; j < 4; ++j)
			for(int k = 0; k < 4; ++k)
			{
				T_mult_temp[i][j] += M1[i][k] * M2[k][j];
			}
	return T_mult_temp;
}

void kdl::log_q(State q) {
	std::ofstream myfile;
	myfile.open("../paths/path.txt");

	myfile << 1 << endl;

	for (int i = 0; i < q.size(); i++)
		myfile << q[i] << " ";
	myfile << endl;

	myfile.close();
}
