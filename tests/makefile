#compiler
OMPL_DIR = /usr/local
INC_CLASSES = ../proj_classes/
INC_PLANNERS = ../planners/
INC_VALIDITY = ../validity_checkers/
INC_RUN = ../run/

EIGEN_DIR = /home/avishai/Documents/eigen

KDL_DIR = /usr/local

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${PQP_DIR}/include $(GL_INCPATH) -I${KDL_DIR}/include  -I$(EIGEN_DIR)  -I$(INC_VALIDITY) -I$(INC_CLASSES)
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -L${KDL_DIR}/lib -lPQP -lm $(GL_LIBS) -larmadillo -lorocos-kdl
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPPQP = ${INC_VALIDITY}collisionDetection.cpp ${INC_VALIDITY}model.cpp
CPPVRF = ${INC_VALIDITY}verification_class.cpp

CPPGDKDL = ${INC_CLASSES}gd_class.cpp ${INC_CLASSES}kdl_class.cpp 
CPPAPC = ${INC_CLASSES}apc_class.cpp ${INC_VALIDITY}StateValidityCheckerPCS.cpp
CPPAPC2 = ${INC_CLASSES}apc_class.cpp 
CPPGD = ${INC_CLASSES}kdl_class.cpp ${INC_VALIDITY}StateValidityCheckerGD.cpp

CPP_P_PCS = ${INC_RUN}plan_PCS.cpp ${INC_PLANNERS}CBiRRT_PCS.cpp

all:
	#$(CXX) test_gd_kdl.cpp ${CPPGD} -o gen $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11	
	#$(CXX) test_gd_kdl_svc.cpp ${CPPGD} ${CPPPQP} -o gen $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11	
	#$(CXX) test_apc.cpp ${CPPAPC2} -o te $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11
	$(CXX) tune_apc.cpp ${CPPAPC2} -o tu $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11
	#$(CXX) test_apc_svc_ik.cpp ${CPPAPC} ${CPPPQP} -o gen $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11
	#$(CXX) test_gd_kdl_svc_rbs.cpp ${CPPGD} ${CPPPQP} -o gen $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11	




