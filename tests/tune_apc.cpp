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

State join(State q1, State q2, State q3) {
    State q(18);
    for (int i = 0; i < 6; i++) {
        q[i] = q1[i];
        q[i+6] = q2[i];
        q[i+12] = q3[i];
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
        
        State q(18);
        State q1(6), q2(6, 0), q3(6, 0);
        //State c_start = {0.01, 0.66, -0.92, 0, 0.29, 0, 0.614271, -0.42148, 0.291163, 1.73151, -0.613028, -1.8028, -0.616349, -0.36761, 0.255771, -1.69897, -0.632059, 1.76603 };        
        q1 = {0.01, 0.66, -0.92, 0, 0.29, 0};
        q2 = {0.614271, -0.42148, 0.291163, 1.73151, -0.613028, -1.8028};
        q3 = {-0.616349, -0.36761, 0.255771, -1.69897, -0.632059, 1.76603};
        q = join(q1, q2, q3);
        A.log_q(q);
        if (!A.check_angle_limits(q)) 
            cout << "Breach of joint limits!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        
        int ik_sol1 = 1, ik_sol2 = 1;
        
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
                case '9':
                if (ik_sol1 < 7)
                ik_sol1++;
                cout << "ik_sol1 = " << ik_sol1 << endl;
                break;
                case 'o':
                if (ik_sol1 > 0)
                ik_sol1--;
                cout << "ik_sol1 = " << ik_sol1 << endl;
                break;
                case '0':
                if (ik_sol2 < 7)
                ik_sol2++;
                cout << "ik_sol2 = " << ik_sol2 << endl;
                break;
                case 'p':
                if (ik_sol2 > 0)
                ik_sol2--;
                cout << "ik_sol2 = " << ik_sol2 << endl;
                break;
                default:
                break;
            }
            cout << "q1: "; A.printVector(q1);
            
            bool flag;
            flag = A.calc_specific_IK_solution_R1(q1, ik_sol1, ik_sol2);
            if (flag) {
                q2 = A.get_IK_solution_q2();
                q3 = A.get_IK_solution_q3();
                cout << "IK success with IK solution #" << ik_sol1 << " and # " << ik_sol2 << endl;
                cout << "q2: "; A.printVector(q2);
                cout << "q3: "; A.printVector(q3);
            }
            q = join(q1, q2, q3);
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
    
    