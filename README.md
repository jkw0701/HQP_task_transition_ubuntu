# HQP_task_transition

* This project is the controller for task transition based on Hierarchical Quadratic Programming(HQP).
* This is developed in Ubuntu 16.04

## Dependencies
* qpOASES
* RBDL(Rigid Body Dynamics Library)
* V-REP


## RBDL Setup 

### Installing
```sh
wget https://bitbucket.org/rbdl/rbdl/get/default.zip
unzip default.zip
cd rbdl-rbdl-0879ee8c548a
mkdir build
cd build
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make all
sudo make install
```

## qpOASES setup
Download qpOASES [Link](http://www.qpoases.org/go/release) 
```sh
cd qpOASES-3.2.1
mkdir build
cd build
cmake ..
make all
sudo make install
```

### qpOASES error handling
if error occures, add following line to qpOASES-3.2.1/CMakeLists.txt, below PROJECT(qpOASES CXX), which is line 34

```
add_compile_options(-fPIC)
```


## VREP setup
### How to start with VREP ###
clone mujoco_ros_sim, then build. 
* git clone https://github.com/saga702/mujoco_ros_sim

after build, launch simulation.launch 
```sh
roslaunch dyros_red_launch simulation.launch 
```
