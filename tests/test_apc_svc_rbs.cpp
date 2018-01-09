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

	State qa1(6), qa2(6), qa3(6);
	State qb1(6), qb2(6), qb3(6);
	Matrix ik_a = {{-1, -1},{-1, -1},{-1, -1}};
	Matrix ik_b = {{-1, -1},{-1, -1},{-1, -1}};
	int ac;

	while(1) {
		State qa = svc.sample_q();//{-0.331011, 1.68225, -0.699434, 2.0305, 1.7584, -1.55728, 0.26575, 0.945871, 0.25798, 1.52788, 0.380337, -2.41114, -0.530635, 1.42369, 0.400287, 1.0527, 1.95218, 1.90697};
		State qb = svc.sample_q();//{-0.32811, 1.6519, -0.650934, 2.01913, 1.73349, -1.54242, 0.270397, 0.930976, 0.280842, 1.57787, 0.339469, -2.4823, -0.518861, 1.4362, 0.377871, 1.04294, 2.00077, 1.8968};
		
		svc.seperate_Vector(qa, qa1, qa2, qa3);
		ik_a = svc.identify_state_ik(qa1, qa2, qa3);
		svc.seperate_Vector(qb, qb1, qb2, qb3);
		ik_b = svc.identify_state_ik(qb1, qb2, qb3);

		for (int i = 0; i < 3; i++) {
			ac = i;
			if (ik_a[i][0] == ik_b[i][0] && ik_a[i][1] == ik_b[i][1]  && svc.checkMotionRBS(qa1, qa2, qa3, qb1, qb2, qb3, ac, ik_a[ac], 0, 0))
				break;
		}
	}

	Matrix Confs;
	Confs.push_back(svc.join_Vectors(qa1, qa2, qa3));
	Confs.push_back(svc.join_Vectors(qb1, qb2, qb3));

	svc.reconstructRBS(qa1, qa2, qa3, qb1, qb2, qb3, ac, ik_a[ac], Confs, 0, 1, 1);
	svc.printMatrix(Confs);


	// svc.two_robots::log_q(qa);
	// //cin.ignore();
	// svc.two_robots::log_q(qb);
	

}