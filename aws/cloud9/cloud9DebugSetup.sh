#!/bin/bash
mkdir -p ~/environment/.c9/runners
echo 'Creating run files...'
cd /data/acs/ginan/examples/
YAMLS="*pea*.yaml"
for yaml in $YAMLS
do
  cat <<EOF >~/environment/.c9/runners/$yaml.run
  {  
    "\$debugDefaultState": true,
    "info": "Running cpp \$file",
    "script": [
      "mkdir -p /data/acs/ginan/src/build/",
      "cd /data/acs/ginan/src/build/",
      "cmake ../",
      "make -j8 pea",
    
    //edit this line to set executable to run  
      "cd ../../examples/",
      "pwd",
      "echo to run",
      "node ~/.c9/bin/c9gdbshim.js /data/acs/ginan/bin/pea --config $yaml"
      
    ],
    "working_dir": "/data/acs/ginan/examples/",
    
    "debugger": "gdb",
    
    "trackId": "Cplusplus"
  }
  // This file overrides the built-in AWS C++ runner
  // For more information see http://docs.aws.amazon.com/console/cloud9/change-runner
EOF
done


echo 'Done'
echo ''
echo 'To debug: Open a cpp file,'
echo 'Click Run - Run With - (Wait) - The yaml configuration of choice'
echo 'Will compile and run in debug mode'
echo ''
echo 'edit ~/environment/.c9/runners/XXX.run to set config options if needed'
echo ''
echo 'Add breakpoints by clicking to left of line numbers in cpp files'
echo 'Debug things are on the right under the bug icon'


