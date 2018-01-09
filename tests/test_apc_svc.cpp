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

	State q(12);// = svc.sample_q();
	q = {-0.04, 0.33, -0.05, 0, 1.2908, -1.6208, 0.00538822, 0.743805, -0.445341, -0.0157073, -0.298499, 0.0150128}; // On the conveyor  
	//q = {1.57, 0.55, 0.73, 0, 0.2908, 0.0092, -1.19386, 1.0596, 0.00106656, -1.22915, -1.38548, 0.478077};  // Mounted on the chassis
	svc.printVector(q);
	cout << svc.collision_state(q) << endl;

	svc.two_robots::log_q(q);

}