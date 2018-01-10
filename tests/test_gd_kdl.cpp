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
	int n = 18;//K.get_numJoints();

	State q(n,0), qp_kdl(n);
	double kdl_time = 0;

	// // q = {-0.271874, 1.5237, -1.14962, 1.05946, -0.461563, -1.41363, 0.738741, 1.41063, -0.703244, 1.12245, -1.75024, -0.744179, 1.97345, -0.258133, 2.29856, 2.39699, -0.875349, 0.679772};
	
	// for (int j = 0; j < n; j++)
	// 	q[j] = /*q[j] * 1.01;*/fRand(-2*3.14, 2*3.14);
	// cout << "qBuild: "; K.printVector(q);

	// K.FK(q, 1);
	// MAtrix T1 = K.get_FK_solution();
	// cout << "T1:\n";K.printMatrix(T1);
	// K.set_Tpose12(T1);
	
	// K.FK(q, 2);
	// MAtrix T2 = K.get_FK_solution();
	// cout << "T2:\n";K.printMatrix(T2);
	// K.set_Tpose13(T2);

	// for (int j = 0; j < n; j++)
	// 	 q[j] = /*q[j] * 1.01;*/fRand(-2*3.14, 2*3.14);
	// cout << "qInit: "; K.printVector(q);

		bool Ksuc = false;
		while (!Ksuc) {

			for (int j = 0; j < n; j++)
				 q[j] = /*q[j] * 1.01;*/fRand(-2*3.14, 2*3.14);

			// KDL
			clock_t begin = clock();
			Ksuc = K.GD(q);
			qp_kdl = K.get_GD_result();
			kdl_time = double(clock() - begin) / CLOCKS_PER_SEC;
		}

		cout << "Success: " << Ksuc << ", sample time: " << kdl_time << endl;

		cout << "qp_kdl: "; K.printVector(qp_kdl);
		K.log_q(qp_kdl);

		// K.FK(qp_kdl, 1);
		// cout << "T1_kdl:\n";K.printMatrix(K.get_FK_solution());
		
		// K.FK(qp_kdl, 2);
		// cout << "T2_kdl:\n";K.printMatrix(K.get_FK_solution());

}