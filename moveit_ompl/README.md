# moveit_ompl

Custom interface for the OMPL in MoveIt

## Build Note

Make sure in your CMakeLists.txt of any package that includes this library's header files, to add Boost's serialization library. It is not header-only! E.g.:

    find_package(Boost REQUIRED system serialization)
