declare_PID_Component(
    SHARED_LIB
    NAME eigen-cdd
    DIRECTORY eigen-cdd
    CXX_STANDARD 11
)

declare_PID_Component_Dependency(
    COMPONENT eigen-cdd
    EXPORT EXTERNAL eigen
    INCLUDE_DIRS <eigen>/include
)

declare_PID_Component_Dependency(
    COMPONENT eigen-cdd
    EXPORT NATIVE cdd
    PACKAGE cdd
)