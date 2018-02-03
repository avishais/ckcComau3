#include "collisionDetection.h"

#define CADLINK "./simulator/"


void pm(PQP_REAL M[][3], std::string str) {
	std::cout << str << std::endl;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (fabs(M[i][j])>1e-4)
				std::cout << M[i][j] << " ";
			else
				std::cout << 0 << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
}

void pv(PQP_REAL V[3], std::string str) {
	std::cout << str << std::endl;

	for (int i = 0; i < 3; i++)
		std::cout << V[i] << " ";
	std::cout << std::endl;
}

int collisionDetection::collision_state(State q) {
	State q1(6), q2(6), q3(6);
	for (int i = 0; i < 6; i++) {
		q1[i] = q[i];
		q2[i] = q[i+6];
		q3[i] = q[i+12];
	}
	return collision_state(q1, q2, q3);
}


int collisionDetection::collision_state(State q1, State q2, State q3)
// Returns 0 if no collision
{
	collisionCheck_counter++;
	clock_t begin = clock();

	double rot1 = q1[0];
	double rot2 = q1[1];
	double rot3 = q1[2];
	double rot4 = q1[3];
	double rot5 = q1[4];
	double rot6 = q1[5];
	double rot12 = q2[0];
	double rot22 = q2[1];
	double rot32 = q2[2];
	double rot42 = q2[3];
	double rot52 = q2[4];
	double rot62 = q2[5];
	double rot13 = q3[0];
	double rot23 = q3[1];
	double rot33 = q3[2];
	double rot43 = q3[3];
	double rot53 = q3[4];
	double rot63 = q3[5];

	// ----------------------- ROBOT 1 -----------------------------

	// make items for transformations
	PQP_REAL R0[3][3],R1[3][3],R2[3][3],T0[3],T1[3],T2[3];
	PQP_REAL R3[3][3],R4[3][3],R5[3][3],T3[3],T4[3],T5[5];
	PQP_REAL R6[3][3],R7[3][3],T6[3],T7[3],T2_t[3], Tt[3], Twheel[3];

	PQP_REAL M0[3][3],M1[3][3],M2[3][3],M3[3][3],M4[3][3];
	PQP_REAL M5[3][3],M6[3][3],M7[3][3], Mwheel[3][3], Rwheel[3][3];

	double oglm[16];

	// rotation matrix
	MRotZ(M0,ROBOT1_ROT);     //base rotate Z
	MxM(R0,M0,M0);

	MRotZ(M1,rot1);  //link 1 rotate Z
	MxM(R1,R0,M1);

	MRotY(M2,rot2);  //link 2 rotate Y
	MxM(R2,R1,M2);

	MRotY(M3,rot3);  //link 3 rotate Y
	MxM(R3,R2,M3);

	MRotX(M4,rot4);  //link 4 rotate X
	MxM(R4,R3,M4);

	MRotY(M5,rot5);  //link 5 rotate Y
	MxM(R5,R4,M5);

	MRotX(M6,rot6);  //link 6 rotate X
	MxM(R6,R5,M6);

	MRotY(M7,0);
	MxM(R7,R6,M7);

	//define kinematics

	T0[0] =  ROBOT1_X;
	T0[1] =  ROBOT1_Y;
	T0[2] =  ROBOT1_Z;

	MxV(T0,R0,T0);

	T1[0] =  0;
	T1[1] =  0;
	T1[2] =  344;

	MxV(Tt,R0,T1);
	VpV(T1,T0,Tt);

	T2[0] =  524.5;
	T2[1] =  -183.5;
	T2[2] =  199;

	MxV(Tt,R1,T2);
	VpV(T2,T1,Tt);

	T2_t[0] = T2[0];
	T2_t[1] = T2[1];
	T2_t[2] = T2[2];

	T2[0] =  0;
	T2[1] =  0;
	T2[2] =  1250;

	MxV(T3,R2,T2);
	VpV(T3,T2_t,T3);

	T2[0] =  186.5;
	T2[1] =  183.5;
	T2[2] =  210;

	MxV(T4,R3,T2);
	VpV(T4,T4,T3);

	T2[0] =  1250;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T5,R4,T2);
	VpV(T5,T4,T5);

	T2[0] =  116.5;
	T2[1] =  0;
	T2[2] =  0;

	MxV(T6,R5,T2);
	VpV(T6,T5,T6);

	T2[0] =  22;
	T2[1] =  0;
	T2[2] =  0;

	MxV(Tt,R6,T2);
	VpV(T7,T6,Tt);

	T2[0] =  22 + RD;
	T2[1] =  0;
	T2[2] =  0;

	MRotY(Mwheel,0);
	MxM(Rwheel,R7,Mwheel);
	MxV(Tt,R6,T2);
	VpV(Twheel,T6,Tt);

	// pm(R7, "R7");
	// pv(T7, "T7");
	// pm(Rwheel, "Rwheel");
	// pv(Twheel, "Twheel");


	// ----------------------- ROBOT 2 -----------------------------

	PQP_REAL R02[3][3],R12[3][3],R22[3][3],T02[3],T12[3],T22[3];
	PQP_REAL R32[3][3],R42[3][3],R52[3][3],T32[3],T42[3],T52[5];
	PQP_REAL R62[3][3],R72[3][3],T62[3],T72[3],T2_t2[3];

	PQP_REAL M02[3][3],M12[3][3],M22[3][3],M32[3][3],M42[3][3];
	PQP_REAL M52[3][3],M62[3][3],M72[3][3],MI[3][3];
	MI[0][0]=MI[1][1]=MI[2][2] =1;
	// rotation matrix
	MRotZ(M02,ROBOT2_ROT/2);     //base rotate Z
	MxM(R02,M02,M02);

	MRotZ(M12,rot12);  //link 1 rotate Z
	MxM(R12,R02,M12);

	MRotY(M22,rot22);  //link 2 rotate Y
	MxM(R22,R12,M22);

	MRotY(M32,rot32);  //link 3 rotate Y
	MxM(R32,R22,M32);

	MRotX(M42,rot42);  //link 4 rotate X
	MxM(R42,R32,M42);

	MRotY(M52,rot52);  //link 5 rotate Y
	MxM(R52,R42,M52);

	MRotX(M62,rot62);  //link 6 rotate X
	MxM(R62,R52,M62);

	MRotY(M72,0);
	MxM(R72,R62,M72);

	//define kinematics

	T02[0] =  ROBOT2_X;
	T02[1] =  ROBOT2_Y;
	T02[2] =  ROBOT2_Z;

	// MxV(Tt,R02,T02);
	// T02[0] = Tt[0];
	// T02[1] = Tt[1];
	// T02[2] = Tt[2];

	T12[0] =  0;
	T12[1] =  0;
	T12[2] =  344;

	MxV(Tt,R02,T12);
	VpV(T12,T02,Tt);

	T22[0] =  524.5; 
	T22[1] =  -183.5;
	T22[2] =  199;

	MxV(Tt,R12,T22);
	VpV(T22,T12,Tt);

	T2_t2[0] = T22[0];
	T2_t2[1] = T22[1];
	T2_t2[2] = T22[2];

	T22[0] =  0;
	T22[1] =  0;
	T22[2] =  1250;

	MxV(T32,R22,T22);
	VpV(T32,T2_t2,T32);

	T22[0] =  186.5;
	T22[1] =  183.5;
	T22[2] =  210;

	MxV(T42,R32,T22);
	VpV(T42,T42,T32);

	T22[0] =  1250;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T52,R42,T22);
	VpV(T52,T42,T52);

	T22[0] =  116.5;
	T22[1] =  0;
	T22[2] =  0;

	MxV(T62,R52,T22);
	VpV(T62,T52,T62);

	T22[0] =  22;
	T22[1] =  0.0;
	T22[2] =  0;

	MxV(T72,R62,T22);
	VpV(T72,T62,T72);
	

	// pm(R72, "R72");
	// pv(T72, "T72");

		// ----------------------- ROBOT 3 -----------------------------

	PQP_REAL R03[3][3],R13[3][3],R23[3][3],T03[3],T13[3],T23[3];
	PQP_REAL R33[3][3],R43[3][3],R53[3][3],T33[3],T43[3],T53[5];
	PQP_REAL R63[3][3],R73[3][3],T63[3],T73[3],T2_t3[3];

	PQP_REAL M03[3][3],M13[3][3],M23[3][3],M33[3][3],M43[3][3];
	PQP_REAL M53[3][3],M63[3][3],M73[3][3];
	MI[0][0]=MI[1][1]=MI[2][2] =1;

	// rotation matrix
	MRotZ(M03,ROBOT3_ROT/2);     //base rotate Z
	MxM(R03,M03,M03);

	MRotZ(M13,rot13);  //link 1 rotate Z
	MxM(R13,R03,M13);

	MRotY(M23,rot23);  //link 2 rotate Y
	MxM(R23,R13,M23);

	MRotY(M33,rot33);  //link 3 rotate Y
	MxM(R33,R23,M33);

	MRotX(M43,rot43);  //link 4 rotate X
	MxM(R43,R33,M43);

	MRotY(M53,rot53);  //link 5 rotate Y
	MxM(R53,R43,M53);

	MRotX(M63,rot63);  //link 6 rotate X
	MxM(R63,R53,M63);

	MRotY(M73,0);
	MxM(R73,R63,M73);

	//define kinematics

	T03[0] =  ROBOT3_X;
	T03[1] =  ROBOT3_Y;
	T03[2] =  ROBOT3_Z;

	// MxV(Tt,R02,T02);
	// T02[0] = Tt[0];
	// T02[1] = Tt[1];
	// T02[2] = Tt[2];

	T13[0] =  0;
	T13[1] =  0;
	T13[2] =  344;

	MxV(Tt,R03,T13);
	VpV(T13,T03,Tt);

	T23[0] =  524.5; 
	T23[1] =  -183.5;
	T23[2] =  199;

	MxV(Tt,R13,T23);
	VpV(T23,T13,Tt);

	T2_t3[0] = T23[0];
	T2_t3[1] = T23[1];
	T2_t3[2] = T23[2];

	T23[0] =  0;
	T23[1] =  0;
	T23[2] =  1250;

	MxV(T33,R23,T23);
	VpV(T33,T2_t3,T33);

	T23[0] =  186.5;
	T23[1] =  183.5;
	T23[2] =  210;

	MxV(T43,R33,T23);
	VpV(T43,T43,T33);

	T23[0] =  1250;
	T23[1] =  0;
	T23[2] =  0;

	MxV(T53,R43,T23);
	VpV(T53,T43,T53);

	T23[0] =  116.5;
	T23[1] =  0;
	T23[2] =  0;

	MxV(T63,R53,T23);
	VpV(T63,T53,T63);

	T23[0] =  22;
	T23[1] =  0.0;
	T23[2] =  0;

	MxV(T73,R63,T23);
	VpV(T73,T63,T73);

	// pm(R23, "R23");
	// pv(T23, "T23");
	// pv(T2_t3, "T2_t3");
	// pm(R33, "R33");
	// pv(T33, "T33");
	// pm(R43, "R43");
	// pv(T43, "T43");
	// pm(R53, "R53");
	// pv(T53, "T53");
	// pm(R63, "R63");
	// pv(T63, "T63");
	// pv(T73, "T63");

	// pm(R63, "R63");
	// pv(T63, "T63");

	// perform tolerance query
	PQP_REAL tolerance = 8;
	PQP_ToleranceResult res[200];
	int i = 0;

	// robot 1 collision
	PQP_Tolerance(&res[i],R0,T0,&base,R4,T4,&link4,tolerance); i++;
	PQP_Tolerance(&res[i],R1,T1,&link1,R4,T4,&link4,tolerance); i++;
	PQP_Tolerance(&res[i],R2,T2_t,&link2,R4,T4,&link4,tolerance); i++;
	PQP_Tolerance(&res[i],R0,T0,&base,R5,T5,&link5,tolerance); i++;
	PQP_Tolerance(&res[i],R1,T1,&link1,R5,T5,&link5,tolerance); i++;
	PQP_Tolerance(&res[i],R2,T2_t,&link2,R5,T5,&link5,tolerance); i++;
	PQP_Tolerance(&res[i],R0,T0,&base,R6,T6,&link6,tolerance); i++;
	PQP_Tolerance(&res[i],R1,T1,&link1,R6,T6,&link6,tolerance); i++;
	PQP_Tolerance(&res[i],R2,T2_t,&link2,R6,T6,&link6,tolerance); i++;
	PQP_Tolerance(&res[i],R0,T0,&base,R3,T3,&link3,tolerance); i++;
	// PQP_Tolerance(&res[i],R0,T0,&base,R6,T7,&EE,tolerance); i++;
	// PQP_Tolerance(&res[i],R1,T1,&link1,R6,T7,&EE,tolerance); i++;
	// PQP_Tolerance(&res[i],R2,T2_t,&link2,R6,T7,&EE,tolerance); i++;
	PQP_Tolerance(&res[i],R0,T0,&base,R2,T2_t,&link2,tolerance); i++;  //13
	
	// robot 2 collision
	PQP_Tolerance(&res[i],R02,T02,&base2,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],R12,T12,&link12,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],R22,T2_t2,&link22,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],R02,T02,&base2,R52,T52,&link52,tolerance); i++;
	PQP_Tolerance(&res[i],R12,T12,&link12,R52,T52,&link52,tolerance); i++;
	PQP_Tolerance(&res[i],R22,T2_t2,&link22,R52,T52,&link52,tolerance); i++;
	PQP_Tolerance(&res[i],R02,T02,&base2,R62,T62,&link62,tolerance); i++;
	PQP_Tolerance(&res[i],R12,T12,&link12,R62,T62,&link62,tolerance); i++;
	PQP_Tolerance(&res[i],R22,T2_t2,&link22,R62,T62,&link62,tolerance); i++;
	PQP_Tolerance(&res[i],R02,T02,&base2,R32,T32,&link32,tolerance); i++;
	// PQP_Tolerance(&res[i],R02,T02,&base2,R62,T72,&EE2,tolerance); i++;
	// PQP_Tolerance(&res[i],R12,T12,&link12,R62,T72,&EE2,tolerance); i++;
	// PQP_Tolerance(&res[i],R22,T2_t2,&link22,R62,T72,&EE2,tolerance); i++;
	PQP_Tolerance(&res[i],R02,T02,&base2,R22,T2_t2,&link22,tolerance); i++;	//27

	// robot 3 collision
	PQP_Tolerance(&res[i],R03,T03,&base3,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R13,T13,&link13,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R23,T2_t3,&link23,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R03,T03,&base3,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R13,T13,&link13,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R23,T2_t3,&link23,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R03,T03,&base3,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R13,T13,&link13,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R23,T2_t3,&link23,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R03,T03,&base3,R33,T33,&link33,tolerance); i++;
	// PQP_Tolerance(&res[i],R03,T03,&base3,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R13,T13,&link13,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R23,T2_t3,&link23,R63,T73,&EE3,tolerance); i++;
	PQP_Tolerance(&res[i],R03,T03,&base3,R23,T2_t3,&link23,tolerance); i++;	//41

	// inter-robot collision link2 and up for robot 1 & 2
	PQP_Tolerance(&res[i],R3,T3,&link3,R32,T32,&link32,tolerance); i++;
	PQP_Tolerance(&res[i],R3,T3,&link3,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],R3,T3,&link3,R52,T52,&link52,tolerance); i++;
	PQP_Tolerance(&res[i],R3,T3,&link3,R62,T62,&link62,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R32,T32,&link32,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R52,T52,&link52,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R62,T62,&link62,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R32,T32,&link32,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R52,T52,&link52,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R62,T62,&link62,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R32,T32,&link32,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R52,T52,&link52,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R62,T62,&link62,tolerance); i++; //57

	// inter-robot collision link2 and up for robot 1 & 3
	PQP_Tolerance(&res[i],R3,T3,&link3,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R3,T3,&link3,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R3,T3,&link3,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R3,T3,&link3,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R4,T4,&link4,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R5,T5,&link5,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R6,T6,&link6,R63,T63,&link63,tolerance); i++; //73

	// inter-robot collision link2 and up for robot 2 & 3
	PQP_Tolerance(&res[i],R32,T32,&link32,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R32,T32,&link32,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R32,T32,&link32,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R32,T32,&link32,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R42,T42,&link42,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R42,T42,&link42,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R42,T42,&link42,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R42,T42,&link42,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R52,T52,&link52,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R52,T52,&link52,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R52,T52,&link52,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R52,T52,&link52,R63,T63,&link63,tolerance); i++;
	PQP_Tolerance(&res[i],R62,T62,&link62,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],R62,T62,&link62,R43,T43,&link43,tolerance); i++;
	PQP_Tolerance(&res[i],R62,T62,&link62,R53,T53,&link53,tolerance); i++;
	PQP_Tolerance(&res[i],R62,T62,&link62,R63,T63,&link63,tolerance); i++; //89

	//inter EE collisions
	// PQP_Tolerance(&res[i],R3,T3,&link3,R62,T72,&EE2,tolerance); i++;
	// PQP_Tolerance(&res[i],R4,T4,&link4,R62,T72,&EE2,tolerance); i++;
	// PQP_Tolerance(&res[i],R5,T5,&link5,R62,T72,&EE2,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T6,&link6,R62,T72,&EE2,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R32,T32,&link32,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R42,T42,&link42,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R52,T52,&link52,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R62,T62,&link62,tolerance); i++; //97

	// PQP_Tolerance(&res[i],R3,T3,&link3,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R4,T4,&link4,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R5,T5,&link5,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T6,&link6,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R33,T33,&link33,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R43,T43,&link43,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R53,T53,&link53,tolerance); i++;
	// PQP_Tolerance(&res[i],R6,T7,&EE,R63,T63,&link63,tolerance); i++; //105

	// PQP_Tolerance(&res[i],R32,T32,&link32,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R42,T42,&link42,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R52,T52,&link52,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R62,T62,&link62,R63,T73,&EE3,tolerance); i++;
	// PQP_Tolerance(&res[i],R62,T72,&EE2,R33,T33,&link33,tolerance); i++;
	// PQP_Tolerance(&res[i],R62,T72,&EE2,R43,T43,&link43,tolerance); i++;
	// PQP_Tolerance(&res[i],R62,T72,&EE2,R53,T53,&link53,tolerance); i++;
	// PQP_Tolerance(&res[i],R62,T72,&EE2,R63,T63,&link63,tolerance); i++; //113

	// robot collision with wheel
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R0,T0,&base,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R1,T1,&link1,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R2,T2_t,&link2,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R3,T3,&link3,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R4,T4,&link4,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R02,T02,&base2,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R12,T12,&link12,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R22,T2_t2,&link22,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R32,T32,&link32,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R42,T42,&link42,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R03,T03,&base3,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R13,T13,&link13,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R23,T2_t3,&link23,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R33,T33,&link33,tolerance); i++;
	PQP_Tolerance(&res[i],Rwheel,Twheel,&wheel,R43,T43,&link43,tolerance); i++; //128

	int obs_max_index = i-1;

	// Collision with obstacles
	if (withObs && env == 1) {

		PQP_REAL Mobs[3][3], Robs[3][3], Tobs[3];

		// Collision with obs
		MRotZ(Robs,0);

		Tobs[0] =  600;
		Tobs[1] =  0;
		Tobs[2] =  1300;
		
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R4,T4,&link4,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R5,T5,&link5,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R6,T6,&link6,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R32,T32,&link32,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R42,T42,&link42,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R52,T52,&link52,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R62,T62,&link62,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R33,T33,&link33,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R43,T43,&link43,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R53,T53,&link53,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,R63,T63,&link63,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs,Rwheel,Twheel,&wheel,tolerance); i++;

		// // Collision with obs1
		MRotZ(Robs,0);
		
		Tobs[0] =  -850;
		Tobs[1] =  300;
		Tobs[2] =  0;
		
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R3,T3,&link3,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R4,T4,&link4,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R5,T5,&link5,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R6,T6,&link6,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R32,T32,&link32,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R42,T42,&link42,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R52,T52,&link52,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R62,T62,&link62,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R33,T33,&link33,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R43,T43,&link43,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R53,T53,&link53,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,R63,T63,&link63,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs1,Rwheel,Twheel,&wheel,tolerance); i++;

		// // Collision with obs2
		MRotZ(Robs,0);
		
		Tobs[0] =  120;
		Tobs[1] =  1070;
		Tobs[2] =  0;
		
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R4,T4,&link4,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R5,T5,&link5,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R6,T6,&link6,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R22,T22,&link22,tolerance); i++;		
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R32,T32,&link32,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R42,T42,&link42,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R52,T52,&link52,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R62,T62,&link62,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R43,T43,&link43,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R53,T53,&link53,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,R63,T63,&link63,tolerance); i++;
		PQP_Tolerance(&res[i],Robs,Tobs,&obs2,Rwheel,Twheel,&wheel,tolerance); i++;

		obs_max_index = i-1;
	}

	clock_t end = clock();
	collisionCheck_time += double(end - begin) / CLOCKS_PER_SEC;

	// if any parts in collision
	for (int j = 0; j <= obs_max_index; ++j) {
		if (res[j].CloserThanTolerance() == 1){
			//std::cout << "Collision in " << j << std::endl;
			return 1;
		}
	}

	// else not in collision
	return 0;
}

void collisionDetection::load_models(){
	FILE *fp;
	int i, ntris;

	// ------------------------- ROBOT 1 ---------------------------

	// initialize the base

	fp = fopen(CADLINK "base_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open base_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::base.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::base.AddTri(p1,p2,p3,i);
	}
	collisionDetection::base.EndModel();
	fclose(fp);

	// initialize link 1

	fp = fopen(CADLINK "link1_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link1_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link1.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link1.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link1.EndModel();
	fclose(fp);

	// initialize link2

	fp = fopen(CADLINK "link2_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link2_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link2.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link2.EndModel();
	fclose(fp);

	// initialize link3

	fp = fopen(CADLINK "link3_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link3_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link3.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link3.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link3.EndModel();
	fclose(fp);

	// initialize link4

	fp = fopen(CADLINK "link4_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link4_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link4.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link4.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link4.EndModel();
	fclose(fp);

	// initialize link5

	fp = fopen(CADLINK "link5_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link5_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link5.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link5.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link5.EndModel();
	fclose(fp);

	// initialize link6

	fp = fopen(CADLINK "link6.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link6.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link6.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link6.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link6.EndModel();
	fclose(fp);

	// initialize EE
	// fp = fopen(CADLINK "ee.tris","r");
	// if (fp == NULL) { fprintf(stderr,"Couldn't open ee.tris\n"); exit(-1); }
	// fscanf(fp,"%d",&ntris);

	// collisionDetection::EE.BeginModel();
	// for (i = 0; i < ntris; i++)
	// {
	// 	double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
	// 	fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
	// 			&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
	// 	PQP_REAL p1[3],p2[3],p3[3];
	// 	p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
	// 	p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
	// 	p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
	// 	collisionDetection::EE.AddTri(p1,p2,p3,i);
	// }
	// collisionDetection::EE.EndModel();
	// fclose(fp);

	// ------------------------- ROBOT 2 ---------------------------

	fp = fopen(CADLINK "base_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open base_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::base2.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::base2.AddTri(p1,p2,p3,i);
	}
	collisionDetection::base2.EndModel();
	fclose(fp);

	// initialize link 1

	fp = fopen(CADLINK "link1_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link1_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link12.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link12.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link12.EndModel();
	fclose(fp);

	// initialize link2

	fp = fopen(CADLINK "link2_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link2_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link22.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link22.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link22.EndModel();
	fclose(fp);

	// initialize link3

	fp = fopen(CADLINK "link3_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link3_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link32.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link32.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link32.EndModel();
	fclose(fp);

	// initialize link4

	fp = fopen(CADLINK "link4_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link4_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link42.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link42.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link42.EndModel();
	fclose(fp);

	// initialize link5

	fp = fopen(CADLINK "link5_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link5_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link52.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link52.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link52.EndModel();
	fclose(fp);

	// initialize link6

	fp = fopen(CADLINK "link6.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link6.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link62.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link62.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link62.EndModel();
	fclose(fp);

	// initialize EE

	// fp = fopen(CADLINK "ee.tris","r");
	// if (fp == NULL) { fprintf(stderr,"Couldn't open ee.tris\n"); exit(-1); }
	// fscanf(fp,"%d",&ntris);

	// collisionDetection::EE2.BeginModel();
	// for (i = 0; i < ntris; i++)
	// {
	// 	double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
	// 	fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
	// 			&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
	// 	PQP_REAL p1[3],p2[3],p3[3];
	// 	p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
	// 	p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
	// 	p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
	// 	collisionDetection::EE2.AddTri(p1,p2,p3,i);
	// }
	// collisionDetection::EE2.EndModel();
	// fclose(fp);

		// ------------------------- ROBOT 3 ---------------------------

	fp = fopen(CADLINK "base_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open base_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::base3.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::base3.AddTri(p1,p2,p3,i);
	}
	collisionDetection::base3.EndModel();
	fclose(fp);

	// initialize link 1

	fp = fopen(CADLINK "link1_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link1_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link13.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link13.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link13.EndModel();
	fclose(fp);

	// initialize link2

	fp = fopen(CADLINK "link2_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link2_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link23.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link23.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link23.EndModel();
	fclose(fp);

	// initialize link3

	fp = fopen(CADLINK "link3_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link3_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link33.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link33.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link33.EndModel();
	fclose(fp);

	// initialize link4

	fp = fopen(CADLINK "link4_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link4_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link43.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link43.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link43.EndModel();
	fclose(fp);

	// initialize link5

	fp = fopen(CADLINK "link5_r.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link5_r.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link53.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link53.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link53.EndModel();
	fclose(fp);

	// initialize link6

	fp = fopen(CADLINK "link6.tris","r");
	if (fp == NULL) { fprintf(stderr,"Couldn't open link6.tris\n"); exit(-1); }
	fscanf(fp,"%d",&ntris);

	collisionDetection::link63.BeginModel();
	for (i = 0; i < ntris; i++)
	{
		double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
		fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
		PQP_REAL p1[3],p2[3],p3[3];
		p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
		p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
		p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
		collisionDetection::link63.AddTri(p1,p2,p3,i);
	}
	collisionDetection::link63.EndModel();
	fclose(fp);

	// initialize EE

	// fp = fopen(CADLINK "ee.tris","r");
	// if (fp == NULL) { fprintf(stderr,"Couldn't open ee.tris\n"); exit(-1); }
	// fscanf(fp,"%d",&ntris);

	// collisionDetection::EE2.BeginModel();
	// for (i = 0; i < ntris; i++)
	// {
	// 	double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
	// 	fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
	// 			&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
	// 	PQP_REAL p1[3],p2[3],p3[3];
	// 	p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
	// 	p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
	// 	p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
	// 	collisionDetection::EE2.AddTri(p1,p2,p3,i);
	// }
	// collisionDetection::EE2.EndModel();
	// fclose(fp);

	// ------------------------------------------------------

			// initialize b1
			fp = fopen(CADLINK "wheel.tris","r");
			if (fp == NULL) { fprintf(stderr,"Couldn't open wheel.tris\n"); exit(-1); }
			fscanf(fp,"%d",&ntris);

			collisionDetection::wheel.BeginModel();
			for (i = 0; i < ntris; i++)
			{
				double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
				fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
						&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
				PQP_REAL p1[3],p2[3],p3[3];
				p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
				p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
				p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
				collisionDetection::wheel.AddTri(p1,p2,p3,i);
			}
			collisionDetection::wheel.EndModel();
			fclose(fp);

	if (withObs) {

		if (env == 1) {

			// initialize obs
			fp = fopen(CADLINK "obs.tris","r");
			if (fp == NULL) { fprintf(stderr,"Couldn't open obs.tris\n"); exit(-1); }
			fscanf(fp,"%d",&ntris);

			collisionDetection::obs.BeginModel();
			for (i = 0; i < ntris; i++)
			{
				double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
				fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
						&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
				PQP_REAL p1[3],p2[3],p3[3];
				p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
				p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
				p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
				collisionDetection::obs.AddTri(p1,p2,p3,i);
			}
			collisionDetection::obs.EndModel();
			fclose(fp);

			// initialize obs1
			fp = fopen(CADLINK "obs1.tris","r");
			if (fp == NULL) { fprintf(stderr,"Couldn't open obs1.tris\n"); exit(-1); }
			fscanf(fp,"%d",&ntris);

			collisionDetection::obs1.BeginModel();
			for (i = 0; i < ntris; i++)
			{
				double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
				fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
						&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
				PQP_REAL p1[3],p2[3],p3[3];
				p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
				p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
				p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
				collisionDetection::obs1.AddTri(p1,p2,p3,i);
			}
			collisionDetection::obs1.EndModel();
			fclose(fp);

			// initialize obs2
			fp = fopen(CADLINK "obs2.tris","r");
			if (fp == NULL) { fprintf(stderr,"Couldn't open obs2.tris\n"); exit(-1); }
			fscanf(fp,"%d",&ntris);

			collisionDetection::obs2.BeginModel();
			for (i = 0; i < ntris; i++)
			{
				double p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z;
				fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",
						&p1x,&p1y,&p1z,&p2x,&p2y,&p2z,&p3x,&p3y,&p3z);
				PQP_REAL p1[3],p2[3],p3[3];
				p1[0] = (PQP_REAL)p1x; p1[1] = (PQP_REAL)p1y; p1[2] = (PQP_REAL)p1z;
				p2[0] = (PQP_REAL)p2x; p2[1] = (PQP_REAL)p2y; p2[2] = (PQP_REAL)p2z;
				p3[0] = (PQP_REAL)p3x; p3[1] = (PQP_REAL)p3y; p3[2] = (PQP_REAL)p3z;
				collisionDetection::obs2.AddTri(p1,p2,p3,i);
			}
			collisionDetection::obs2.EndModel();
			fclose(fp);
		}
	}
}

collisionDetection::collisionDetection() : env(1)
{
	// load the models
	load_models();
	// collisionDetection::offsetX = ROBOTS_DISTANCE_X;
	// collisionDetection::offsetY = ROBOTS_DISTANCE_Y;
	// collisionDetection::offsetZ = 0;
	// collisionDetection::offsetRot = ROBOT2_ROT;

}
