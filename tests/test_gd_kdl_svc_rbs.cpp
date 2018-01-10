#include "StateValidityCheckerGD.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	StateValidityChecker svc(1);

	State qa(18), qb(18);
	while(1) {
		qa = svc.sample_q();//{-0.331011, 1.68225, -0.699434, 2.0305, 1.7584, -1.55728, 0.26575, 0.945871, 0.25798, 1.52788, 0.380337, -2.41114, -0.530635, 1.42369, 0.400287, 1.0527, 1.95218, 1.90697};
		qb = svc.sample_q();//{-0.32811, 1.6519, -0.650934, 2.01913, 1.73349, -1.54242, 0.270397, 0.930976, 0.280842, 1.57787, 0.339469, -2.4823, -0.518861, 1.4362, 0.377871, 1.04294, 2.00077, 1.8968};
		
			if (svc.checkMotionRBS(qa, qb, 0, 0))
				break;
	}

	Matrix Confs;
	Confs.push_back(qa);
	Confs.push_back(qb);

	svc.reconstructRBS(qa, qb, Confs, 0, 1, 1);
	svc.printMatrix(Confs);
	
}