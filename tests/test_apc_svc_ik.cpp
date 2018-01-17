#include "StateValidityCheckerPCS.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

Matrix getPath() {
    const char* robot_pfile = "../paths/path.txt";
    FILE *fr;
    int i, nlines;

    fr = fopen(robot_pfile,"r");
    if (fr == NULL) { fprintf(stderr,"Couldn't open path.txt\n"); exit(-1); }
    fscanf(fr,"%i",&nlines);  //NOT include number in line count itself

    Matrix M;
    for (i = 0; i < nlines; i++)
    {
        double rot1T,rot2T,rot3T,rot4T,rot5T,rot6T;
        double rot52T,rot62T,rot12T,rot22T,rot32T,rot42T;
        double rot53T,rot63T,rot13T,rot23T,rot33T,rot43T;
        fscanf(fr,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                &rot1T,&rot2T,&rot3T,&rot4T,&rot5T,&rot6T, \
                &rot12T,&rot22T,&rot32T,&rot42T,&rot52T,&rot62T, \
                &rot13T,&rot23T,&rot33T,&rot43T,&rot53T,&rot63T);
        M.push_back({rot1T,rot2T,rot3T,rot4T,rot5T,rot6T,rot12T,rot22T,rot32T,rot42T,rot52T,rot62T,rot13T,rot23T,rot33T,rot43T,rot53T,rot63T});
    }

    fclose(fr);
    return M;
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// APC
	StateValidityChecker svc;

    State q(18);
    Matrix M = getPath();
    
    for (int i = 0; i < M.size(); i++) {
        q = M[i];
        
        State q1(6), q2(6), q3(6);
        svc.seperate_Vector(q, q1, q2, q3);
        Matrix ik = svc.identify_state_ik(q1, q2, q3, {{-1, -1},{-1, -1},{-1, -1}});
        svc.printMatrix(ik);

        svc.printVector(q);
        svc.two_robots::log_q(q);
        cout << "-------------------\n";
        cin.ignore();
    
    }




	

}