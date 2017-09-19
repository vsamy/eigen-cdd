
#declare your tests here
declare_PID_Component(
    TEST_APPLICATION
    NAME TestPolyhedron
    DIRECTORY eigen-cdd
)

declare_PID_Component_Dependency(
    COMPONENT TestPolyhedron
    NATIVE eigen-cdd
)

declare_PID_Component_Dependency(
    COMPONENT TestPolyhedron
    EXTERNAL  boost
    INCLUDE_DIRS <boost>/include 
    LINKS SHARED <boost>/lib/libboost_unit_test_framework.so.1.64.0
)

run_PID_Test (NAME test-polyhedron COMPONENT TestPolyhedron)

