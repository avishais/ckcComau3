#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	kdl K;


	int N = 1;
	State q(12,0), qp_kdl(12);
	double kdl_time = 0;

	// K.FK(q);
	// Matrix T = K.get_FK_solution();

	// K.printMatrix(T);

	// for (int i = 0; i < N; i++) {

		for (int j = 0; j < 12; j++)
			q[j] = fRand(-3.14, 3.14);

		// KDL
		clock_t begin = clock();
		bool Ksuc = K.GD(q);
		qp_kdl = K.get_GD_result();
		kdl_time = double(clock() - begin) / CLOCKS_PER_SEC;

		cout << "Success: " << Ksuc << ", sample time: " << kdl_time << endl;

		K.log_q(qp_kdl);

		K.FK(qp_kdl);
		Matrix T = K.get_FK_solution();
	
		K.printMatrix(T);
	// }

}