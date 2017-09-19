
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
    EXTERNAL  boost-utest
    PACKAGE   boost
)

run_PID_Test (NAME test-polyhedron COMPONENT TestPolyhedron)

