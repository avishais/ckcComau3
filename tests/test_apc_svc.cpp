#include "StateValidityCheckerPCS.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// APC
	StateValidityChecker svc;

	State q(18);
	//State q = svc.sample_q();
	//State q = {0.572259, 1.31352, -2.59529, 1.88192, -2.73597, 2.84779, -0.558905, -0.241222, 0.636675, -0.349556, -1.34235, -0.743665, 0.145462, 0.804938, -0.87533, -1.71615, 0.931526, 2.96084}; // On the conveyor  
	//State q1(6), q2(6), q3(6);
	//svc.seperate_Vector(q, q1, q2, q3);
	//Matrix ik = svc.identify_state_ik(q1, q2, q3, {{-1, -1},{-1, -1},{-1, -1}});
	//svc.printMatrix(ik);

	State q1(6), q2(6), q3(6);

	while (1) {
		// Random active chain
		for (int i = 0; i < 18; i++)
			q[i] = ((double) rand() / (RAND_MAX)) * 2 * PI_ - PI_;

		int ik_sol_a = rand() % 8;
		int ik_sol_b = rand() % 8;

		svc.seperate_Vector(q, q1, q2, q3);
		if (svc.IKproject(q1, q2, q3, 2, ik_sol_a, ik_sol_b))
			break;

	}
	q = svc.join_Vectors(q1, q2, q3);


	svc.printVector(q);
	// cout << svc.collision_state(q) << endl;

	svc.two_robots::log_q(q);

}