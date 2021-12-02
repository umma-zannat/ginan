import os

with open("src/CMakeLists.txt", "r") as file:
    filedata = file.read()

# replace the target string
# comment out fortran flags
filedata = filedata.replace(
    'set (CMAKE_Fortran_FLAGS				"-g -frecursive -fall-intrinsics -Wall -fcheck=all -fbacktrace")',
    '',
)
# uncomment github flags
with open("CMakeLists.txt", "w") as file:
    file.write(filedata)

filedata = filedata.replace(
    '#set (CMAKE_Fortran_FLAGS				"-g -frecursive -fall-intrinsics -fcheck=all -fbacktrace")',
    'set (CMAKE_Fortran_FLAGS				"-g -frecursive -fall-intrinsics -Wall -fcheck=all -fbacktrace")',
)

# Write the file out again
with open("CMakeLists.txt", "w") as file:
    file.write(filedata)
import os

with open("./CMakeLists.txt", "r") as file:
    filedata = file.read()

# replace the target string
# comment out fortran flags
filedata = filedata.replace(
    'set(CMAKE_CXX_FLAGS						"${CMAKE_CXX_FLAGS} -std=c++1z -ggdb3 -fpermissive -Wall -Wno-write-strings -Wno-deprecated-declarations -Wno-format-overflow -Wno-c++11-narrowing -Wno-unused-but-set-variable -Wno-sign-compare -Wno-stringop-truncation -Wno-unused-variable")',
    '',
)
# uncomment github flags
with open("CMakeLists.txt", "w") as file:
    file.write(filedata)

filedata = filedata.replace(
    '#set(CMAKE_CXX_FLAGS					"${CMAKE_CXX_FLAGS} -std=c++1z -ggdb3 -fpermissive -Wno-write-strings -Wno-deprecated-declarations -Wno-format-overflow -Wno-c++11-narrowing -Wno-dangling-else -Wno-switch")',
    'set(CMAKE_CXX_FLAGS					"${CMAKE_CXX_FLAGS} -std=c++1z -ggdb3 -fpermissive -Wno-write-strings -Wno-deprecated-declarations -Wno-format-overflow -Wno-c++11-narrowing -Wno-dangling-else -Wno-switch")',
)

# Write the file out again
with open("CMakeLists.txt", "w") as file:
    file.write(filedata)
