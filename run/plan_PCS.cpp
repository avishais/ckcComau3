/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Avishai Sintov, Ioan Sucan */

#include "plan_PCS.h"

bool IKturn = true; // Define whether we sample q1 and solve IK for q2 (true) or vice versa.

bool isStateValid(const ob::State *state)
{
	return true;
}

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
	switch (p_type)
	{
	case PLANNER_BIRRT:
	{
		return std::make_shared<og::CBiRRT>(si, maxStep, env);
		break;
	}
	case PLANNER_RRT:
	{
		return std::make_shared<og::RRT>(si, maxStep, env);
		break;
	}
	case PLANNER_SBL:
	{
		return std::make_shared<og::SBL>(si, maxStep, env);
		break;
	}
	default:
	{
		OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
		return ob::PlannerPtr(); // Address compiler warning re: no return value.
		break;
	}
	}
}

void plan_C::plan(State c_start, State c_goal, double runtime, plannerType ptype, double max_step) {

	// construct the state space we are planning inz
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(18)); // Angles of Robot 1 & 2 - R^18

	// set the bounds for the Q=R^18 part of 'Cspace'
	ob::RealVectorBounds Qbounds(18);
	Qbounds.setLow(0, -PI_); // Robot 1
	Qbounds.setHigh(0, PI_);
	Qbounds.setLow(1, -1.0472);
	Qbounds.setHigh(1, 2.1871);
	Qbounds.setLow(2, -PI_/2);
	Qbounds.setHigh(2, 1.3090);
	Qbounds.setLow(3, -PI_);
	Qbounds.setHigh(3, PI_);
	Qbounds.setLow(4, -2.0944);
	Qbounds.setHigh(4, 2.0944);
	Qbounds.setLow(5, -PI_);
	Qbounds.setHigh(5, PI_);
	Qbounds.setLow(6, -PI_); // Robot 2
	Qbounds.setHigh(6, PI_);
	Qbounds.setLow(7, -1.0472);
	Qbounds.setHigh(7, 2.1871);
	Qbounds.setLow(8, -PI_/2);
	Qbounds.setHigh(8, 1.3090);
	Qbounds.setLow(9, -PI_);
	Qbounds.setHigh(9, PI_);
	Qbounds.setLow(10, -2.0944);
	Qbounds.setHigh(10, 2.0944);
	Qbounds.setLow(11, -PI_);
	Qbounds.setHigh(11, PI_);
	Qbounds.setLow(12, -PI_); // Robot 3
	Qbounds.setHigh(12, PI_);
	Qbounds.setLow(13, -1.0472);
	Qbounds.setHigh(13, 2.1871);
	Qbounds.setLow(14, -PI_/2);
	Qbounds.setHigh(14, 1.3090);
	Qbounds.setLow(15, -PI_);
	Qbounds.setHigh(15, PI_);
	Qbounds.setLow(16, -2.0944);
	Qbounds.setHigh(16, 2.0944);
	Qbounds.setLow(17, -PI_);
	Qbounds.setHigh(17, PI_);

	// set the bound for the compound space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(Q);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.02); // 3% ???

	// create start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Cspace);
	for (int i = 0; i < 18; i++) {
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_start[i]; // Access the first component of the start a-state
	}

	// create goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Cspace);
	for (int i = 0; i < 18; i++) {
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_goal[i]; // Access the first component of the goal a-state
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	maxStep = max_step;
	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner = allocatePlanner(si, ptype);

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	clock_t end = clock();
	cout << "Runtime: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("pathRRTC.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();

		std::cout << "Found solution:" << std::endl;
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}

void extract_from_perf_file(ofstream &ToFile) {
	ifstream FromFile;
	FromFile.open("perf_log.txt");

	string line;
	while (getline(FromFile, line))
		ToFile << line << "\t";

	FromFile.close();
}


int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime;
	plannerType ptype;
	string plannerName;
	int env; // Tested environment index

	if (argn == 1) {
		runtime = 1; // sec
		ptype = PLANNER_BIRRT;
		plannerName = "BiRRT";
		env = 1;
	}
	else if (argn == 2) {
		runtime = atof(args[1]);
		ptype = PLANNER_BIRRT;
		plannerName = "BiRRT";
		env = 1;
	}
	else if (argn > 2) {
		runtime = atof(args[1]);
		switch (atoi(args[2])) {
		case 1 :
			ptype = PLANNER_BIRRT;
			plannerName = "BiRRT";
			break;
		case 2 :
			ptype = PLANNER_RRT;
			plannerName = "RRT";
			break;
		case 3 :
			ptype = PLANNER_SBL;
			plannerName = "SBL";
			break;
		default :
			cout << "Error: Requested planner not defined.";
			exit(1);
		}
		if (argn == 4)
			env = atoi(args[3]);
		else
			env = 1;
	}

	plan_C Plan;

	srand (time(NULL));

	State c_start, c_goal;
	if (env == 1) {
		c_start = {0.01, 0.66, -0.92, 0, 0.29, 0, 0.26018, -0.106253, 0.0485152, 1.73479, -0.253422, -1.76992, -0.274494, -0.0556799, 0.00153614, -1.70184, -0.286787, 1.73772};
		c_goal =  {0.38, 0.65, 0.21, 0, -0.85, 0, -0.527848, -0.0288182, 1.16444, -0.952929, -1.31305, 0.342119, -0.272648, 1.22228, -0.804083, -1.07655, -0.761231, 0.943446};	
		
		Plan.set_environment(1);
	}
	else if (env == 2) {
		Plan.set_environment(2);
	}

	int mode = 3;
	switch (mode) {
	case 1: {
		StateValidityChecker svc(1);
		
		// while (1) {
			// State c_start = {-0.0575635, 1.17677, -1.37513, -0.519196, -0.680525, 1.42335, 0.49729, -0.22321, 0.00143566, 0.319531, 1.12872, 0.103594, -0.867969, -0.139754, 0.234826, 1.62506, 1.5736, -2.64108 };//svc.sample_q();
			// State c_goal = {0.131372, 0.481784, -0.51826, 1.88561, -0.985942, -0.791309, 0.520754, 0.0798232, -0.149879, 2.19257, -1.76884, -2.36709, -0.207683, -0.225008, 0.662886, 0.307782, -1.48048, -0.859081 };//svc.sample_q();
			// State c_start = {0.400374, 1.62944, -1.20526, -1.58796, 0.486744, -0.0653835, 0.73933, 1.58904, 0.776625, 3.05192, -2.05093, 2.87353, -0.693215, 1.7076, -1.44314, -2.28432, -1.68576, 3.05194};
			// State c_goal = {0.530652, 1.14156, 0.550491, 0.997299, -2.05461, 2.91718, -0.15758, 0.884866, 0.306131, -0.583713, 1.39985, -0.458843, -0.208264, 1.53056, -0.805135, -1.67695, -1.77917, -1.77646};
			
			// When initial pole is in x=600, y=0
			State c_start = {0.01, 0.66, -0.92, 0, 0.29, 0, 0.26018, -0.106253, 0.0485152, 1.73479, -0.253422, -1.76992, -0.274494, -0.0556799, 0.00153614, -1.70184, -0.286787, 1.73772};
			
			//State c_goal = {0.465045, 0.716579, -0.00865373, 1.10045, -1.50361, -0.067838, 0.253439, -0.918174, 1.10123, 2.38902, -0.972634, -2.00894, -0.261746, 0.563808, 0.285722, 0.205395, -1.27093, -0.823385};

			// This is with the two obs on the floor when one is the small pole obs2 to go through
			State c_goal =  {0.38, 0.65, 0.21, 0, -0.85, 0, -0.527848, -0.0288182, 1.16444, -0.952929, -1.31305, 0.342119, -0.272648, 1.22228, -0.804083, -1.07655, -0.761231, 0.943446};	

			// When initial pole is in x=0,y=0
			//State c_start = {0, 0.18, -0.48, 0, 0.29, 0, -0.00116218, 0.187689, -0.495204, 0.00387427, 0.312517, 0.00496777, 0.00116218, 0.187689, -0.495204, -0.00387427, 0.312517, -0.00496777};

			// State c_goal = {-0.301608, 0.605662, 0.630057, -0.228812, -1.77406, -1.44587, 0.139727, 1.30729, -0.750739, 0.938481, -1.55819, 0.624497, 0.252422, 0.383411, 0.194915, -0.0332356, 0.973359, 0.816574};	
			
			//State c_goal = {0.229974, 0.371383, -0.9223, -1.04534, 0.914529, 1.178, 0.0421263, 0.562216, -1.36839, 2.52872, -1.22311, -2.91004, 0.0954364, 0.62569, -1.15579, -1.32616, 0.585952, 0.9113};


			//State c_goal = svc.sample_q();

			cout << "------------------------\n";
			State q1(6), q2(6),q3(6);
			
			svc.seperate_Vector(c_start, q1,q2,q3);
			Matrix ik1 = svc.identify_state_ik(q1, q2, q3);
			svc.printMatrix(ik1);
			svc.two_robots::log_q(c_start);
			// cin.ignore();

			svc.seperate_Vector(c_goal, q1,q2,q3);
			Matrix ik2 = svc.identify_state_ik(q1, q2, q3);
			svc.printMatrix(ik2);
			svc.two_robots::log_q(c_goal);
			// cin.ignore();

			// int a0 = (ik1[0][0]==ik2[0][0] && ik1[0][1]==ik2[0][1]);
			// int a1 = (ik1[1][0]==ik2[1][0] && ik1[1][1]==ik2[1][1]);
			// int a2 = (ik1[2][0]==ik2[2][0] && ik1[2][1]==ik2[2][1]);

			// if (a0+a1+a2 > 0)
			// 	continue;

			// cout << svc.check_angle_limits(c_start) << endl;
			// cout << svc.check_angle_limits(c_goal) << endl;
			//Plan.solved_bool = true;

			Plan.plan(c_start, c_goal, runtime, ptype, 1);

		// 	if (Plan.solved_bool)
		// 		break;
		// }

		break;
	}
	case 2 : { // Benchmark planning time with constant maximum step size
		ofstream APS;
		APS.open("./matlab/Benchmark_" + plannerName + "_PCS.txt", ios::app);

		for (int k = 0; k < 100; k++) {
			Plan.plan(c_start, c_goal, runtime, ptype, 0.2);

			// Extract from perf file
			ifstream FromFile;
			FromFile.open("./paths/perf_log.txt");
			string line;
			while (getline(FromFile, line))
				APS << line << "\t";
			FromFile.close();
			APS << endl;
		}
		APS.close();
		break;
	}
	case 3 : { // Benchmark maximum step size while benchmarking the step size
		ofstream APS;
		APS.open("./matlab/Benchmark_" + plannerName + "_PCS_rB.txt", ios::app);

		int N = 100;
		for (int k = 0; k < N; k++) {
			for (int j = 0; j < 10; j++) {
				double maxStep = 0.2 + 0.2*j;

				cout << "** Running PCS iteration " << k << " with maximum step: " << maxStep << " **" << endl;

				Plan.plan(c_start, c_goal, runtime, ptype, maxStep);

				APS << maxStep << "\t";

				// Extract from perf file
				ifstream FromFile;
				FromFile.open("./paths/perf_log.txt");
				string line;
				while (getline(FromFile, line))
					APS << line << "\t";
				FromFile.close();
				APS << endl;
			}
		}
		APS.close();
		break;
	}

	}

	std::cout << std::endl << std::endl;

	return 0;
}

