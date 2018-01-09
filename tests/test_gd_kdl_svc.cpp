#include "StateValidityCheckerGD.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	StateValidityChecker svc(1);

	State q = svc.sample_q();
	svc.printVector(q);

	svc.kdl::log_q(q);
	



}