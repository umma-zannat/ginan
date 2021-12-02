#!/bin/bash

# # ==== Create new Cloud9 environment ====

# # - Login to AWS Cloud9. 
# #     Address: https://ap-southeast-2.console.aws.amazon.com/cloud9/home?region=ap-southeast-2
# # - Click "Create environment". 
# #     Select following options:
# #  - Create a new EC2 instance for environment (direct access)
# #  - m5.large (8 GiB RAM + 2 vCPU) --> or larger
# #  - Ubuntu Server 18.04 LTS
# #  - Cost-saving - after a day


# # ==== Set up directories ====

# # Set up directories + clone repo
# mkdir /home/ubuntu/environment/data 
# sudo ln -s /home/ubuntu/environment/data /data
# mkdir /data/acs 


# # ==== Get Ginan sourcecode with branch containing this file ======

# # Use either the SSH method or HTTPS method below to get Ginan sourcecode:

# # === SSH Method ===
# # == Set up SSH on BitBucket ==
# # Generate SSH key for bitbucket
# ssh-keygen -t rsa
# # Press enter at each terminal prompt (i.e. defaults)
# cat /home/ubuntu/.ssh/id_rsa.pub
# # Paste this public key into Bitbucket - Profile picture (bottom left) -> Personal settings -> SSH keys
# # == Get Ginan ==
# cd /data/acs 
# yes | git clone git@bitbucket.org:geoscienceaustralia/ginan.git 

# # OR

# # === HTTPS Method ===
# # == Get Ginan ==
# # Replace 'user_name' in the url below with your username for bitbucket
# git clone https://user_name@bitbucket.org/geoscienceaustralia/ginan.git
# # A prompt will ask for the password associated with 'user_name', type password and press enter


# # ==== Change branch and obtain this bash script

# cd /data/acs/ginan 
# git fetch && git checkout develop
# cd /data/acs/ginan/aws/cloud9/
# chmod 777 ginan_cloud9_install.sh


# # ==== Re-size, install dependencies, Ginan and python virtual environemnt ====
# ---------> Run this bash script: ./ginan_cloud9_install.sh

# Increase disk size
sudo apt update
yes | sudo apt upgrade
# Enter 'y' when prompted
cd /data/acs/ginan/aws/cloud9/
./resize.sh 200


# Install dependencies
sudo apt install -y git gobjc gobjc++ gfortran libopenblas-dev openssl curl net-tools openssh-server cmake make libssl1.0-dev
sudo mkdir -p /data/tmp
cd /data/tmp 
sudo git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
sudo mkdir cmake-build
cd cmake-build
sudo cmake .. -DCMAKE\_INSTALL\_PREFIX=/usr/local/ -DYAML\_CPP\_BUILD\_TESTS=OFF
sudo make install yaml-cpp
cd ../..
sudo rm -fr yaml-cpp

cd /data/tmp/
sudo wget -c https://boostorg.jfrog.io/artifactory/main/release/1.73.0/source/boost_1_73_0.tar.gz
sudo gunzip boost_1_73_0.tar.gz
sudo tar xvf boost_1_73_0.tar
cd boost_1_73_0/
sudo ./bootstrap.sh
sudo ./b2 install
cd .. 
sudo rm -fr boost_1_73_0/ boost_1_73_0.tar

cd /data/tmp/
sudo git clone https://gitlab.com/libeigen/eigen.git
cd eigen
sudo mkdir cmake-build
cd cmake-build
sudo cmake ..
sudo make install
cd ../..
sudo rm -rf eigen

sudo apt -y install libnetcdf-dev libnetcdf-c++4-dev

# (Alternate one-liner to 'Install dependencies' above)
# sudo apt install -y git gobjc gobjc++ gfortran libopenblas-dev openssl curl net-tools openssh-server cmake make libssl1.0-dev; sudo mkdir -p /data/tmp; cd /data/tmp; sudo git clone https://github.com/jbeder/yaml-cpp.git; cd yaml-cpp; sudo mkdir cmake-build; cd cmake-build; sudo cmake .. -DCMAKE\_INSTALL\_PREFIX=/usr/local/ -DYAML\_CPP\_BUILD\_TESTS=OFF; sudo make install yaml-cpp; cd ../..; sudo rm -fr yaml-cpp; cd /data/tmp/; sudo wget -c https://boostorg.jfrog.io/artifactory/main/release/1.73.0/source/boost_1_73_0.tar.gz; sudo gunzip boost_1_73_0.tar.gz; sudo tar xvf boost_1_73_0.tar; cd boost_1_73_0/; sudo ./bootstrap.sh; sudo ./b2 install; cd ..; sudo rm -fr boost_1_73_0/ boost_1_73_0.tar; cd /data/tmp/; sudo git clone https://gitlab.com/libeigen/eigen.git; cd eigen; sudo mkdir cmake-build; cd cmake-build; sudo cmake ..; sudo make install; cd ../..; sudo rm -rf eigen; sudo apt -y install libnetcdf-dev libnetcdf-c++4-dev;


# Build PEA and POD
cd /data/acs/ginan/src
mkdir -p build
cd build
cmake -DENABLE_OPTIMISATION=TRUE .. >cmake.out 2>cmake.err
make -j 4 >make.out 2>make.err
# 4 = number of threads you want to build with


# Download examples
cd /data/acs/ginan/scripts
python3 download_examples.py

# Run / Test PEA
# cd /data/acs/ginan/examples
# ../bin/pea --config ex11_pea_pp_user_gps.yaml

# Python Virtual Environment Installation
cd /data/tmp
sudo wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -N
bash Miniconda3-latest-Linux-x86_64.sh -b -p /data/miniconda3
eval "$(/data/miniconda3/bin/conda shell.bash hook)"
conda init
conda config --set auto_activate_base false

# deploying conda environment
conda env create -f /data/acs/ginan/scripts/conda_gn37.yaml
# cleaning conda
conda clean -ay

source ~/.bashrc
conda activate gn37




cd /data/tmp
wget https://github.com/mongodb/mongo-c-driver/releases/download/1.17.1/mongo-c-driver-1.17.1.tar.gz
tar -xvf mongo-c-driver-1.17.1.tar.gz

cd mongo-c-driver-1.17.1/
mkdir cmakebuild
cd cmakebuild/
cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF ..
cmake --build .
sudo cmake --build . --target install

cd ../../
curl -OL https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.6.0/mongo-cxx-driver-r3.6.0.tar.gz
tar -xzf mongo-cxx-driver-r3.6.0.tar.gz

cd mongo-cxx-driver-r3.6.0/

cd build/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo cmake --build . --target EP_mnmlstc_core
cmake --build .
sudo cmake --build . --target install

wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list

sudo apt -y update
sudo apt -y install mongodb-org 

sudo systemctl start mongod
sudo systemctl status mongod
mongod

cd ../../
sudo rm -rf mongo-c*
