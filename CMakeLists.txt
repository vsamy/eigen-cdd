CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

PROJECT(eigen-cdd)

declare_PID_Package(	
			AUTHOR 		vsamy
			ADDRESS 	git@github.com:vsamy/eigen-cdd.git
			YEAR 		2017 
			INSTITUTION	LIRMM
			MAIL		vincent.samy@lirmm.fr
			YEAR 		2017 
			LICENSE 	CeCILL-C
			DESCRIPTION "Small Eigen Wrapper of cdd. Based on Fukuda's library named [cdd](https://www.inf.ethz.ch/personal/fukudak/cdd_home)"
)

set_PID_Package_Version(0 4 0)

#now finding packages
declare_PID_Package_Dependency(PACKAGE eigen EXTERNAL VERSION 3.3.4)
declare_PID_Package_Dependency(PACKAGE cdd NATIVE VERSION 0.94)

if(BUILD_AND_RUN_TESTS)
	declare_PID_Package_Dependency(PACKAGE boost EXTERNAL VERSION 1.63.0)
endif()

build_PID_Package()

