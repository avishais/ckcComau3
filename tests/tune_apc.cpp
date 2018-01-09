#include "apc_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include "../proj_classes/def.h"

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

double dist(State p1, State p2) {
	double sum = 0;
	for (int i = 0; i < p1.size(); i++)
		sum += (p1[i]-p2[i])*(p1[i]-p2[i]);

	return sqrt(sum);
}

State join(State q1, State q2) {
    State q(12);
    for (int i = 0; i < 6; i++) {
        q[i] = q1[i];
        q[i+6] = q2[i];
    }
    return q;
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// APC
	two_robots A;

	// State q1(6,0);
	// State q2(6,0);
	// // q = {0.3, -0.5, 0.5, 0, 0.4, 0};
	// q2[0] = 1.5707;
	// A.printVector(q2);

	// A.FKsolve_rob(q2, 2);
	// Matrix T = A.get_FK_solution_T2();
	// A.printMatrix(T);

	// for (int i = 0; i < 8; i++) {
	// 	A.IKsolve_rob(T, 2, i);

	// 	A.printVector(A.get_IK_solution_q2());
	// }
   
    State q(12);
    State q1(6), q2(6, 0);
    //c_start = {-0.04, 0.33, -0.05, 0, 1.2908, -1.6208, 0.00538822, 0.743805, -0.445341, -0.0157073, -0.298499, 0.0150128}; // On the conveyor  
    //c_goal = {1.57, 0.55, 0.73, 0, 0.2908, 0.0092, -1.19386, 1.0596, 0.00106656, -1.22915, -1.38548, 0.478077};  
    //c_start = {-0.03, 0.33, -0.05, 0, 1.2908, -1.5808, 0.0183141, 1.06015, -1.25727, -0.193255, 0.200755, 0.189466};
    //c_goal = {1.57, 0.55, 0.73, 0, 0.2908, 0.0092, -1.12171, 0.868895, -0.0134552, -1.21367, -1.2759, 0.661756};
    q1 = {1.57, 0.55, 0.73, 0, 0.2908, 0.0092};
    q2 = {-1.19386, 1.0596, 0.00106656, -1.22915, -1.38548, 0.478077};
    q = join(q1, q2);
    A.log_q(q);
    if (!A.check_angle_limits(q)) 
        cout << "Breach of joint limits!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

    int ik_sol = 1;

    double dq = 0.01;
	while (1) {

        cout << "Move joints: ";
        char c = getchar();

        switch (c)
        {
            case '1':
            q1[0] += dq;
            break;
            case '2':
            q1[1] += dq;
            break;
            case '3':
            q1[2] += dq;
            break;
            case '4':
            q1[3] += dq;
            break;
            case '5':
            q1[4] += dq;
            break;
            case '6':
            q1[5] += dq;
            break;
            case 'q':
            q1[0] -= dq;
            break;
            case 'w':
            q1[1] -= dq;
            break;
            case 'e':
            q1[2] -= dq;
            break;
            case 'r':
            q1[3] -= dq;
            break;
            case 't':
            q1[4] -= dq;
            break;
            case 'y':
            q1[5] -= dq;
            break;
            case '0':
            if (ik_sol < 7)
                ik_sol++;
            cout << "ik_sol = " << ik_sol << endl;
            break;
            case 'p':
            if (ik_sol > 0)
                ik_sol--;
            cout << "ik_sol = " << ik_sol << endl;
            break;
            default:
            break;
        }
        cout << "q1: "; A.printVector(q1);
      
        bool flag;
        // for (int ik_sol = 0; ik_sol < 8; ik_sol++) {
            flag = A.calc_specific_IK_solution_R1(q1, ik_sol);
            if (flag) {
                cout << "IK success with IK solution #" << ik_sol << endl;
                q2 = A.get_IK_solution_q2();
                cout << "q2: "; A.printVector(q2);	
                // break;
            }
        // }
        q = join(q1, q2);
        A.log_q(q);
        if (flag) {
            cout << "q: "; A.printVector(q);
            if (!A.check_angle_limits(q))
		        cout << "Breach of joint limits!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        }
        else {
            q2 = {0,0,0,0,0,0};
            cout << "IK failed." << endl;
            //continue;
        }
		
    }
}

