# franka_research_ws

## INSTALLATION
Clone the following repo:

```
git clone https://github.com/tasbolat1/franka_research_ws.git --recursive
```

### Libfranka
Go to the libfranka, set branch to 0.10.0 and update it
```
cd ~/franka_research_ws/src/libfranka
git checkout 0.10.0
cd ~/franka_research_ws
git submodule update --init --recursive --remote
git checkout 4f9e3cc666e42d267f1ab566869c4f4c552e5b57
cd common
git checkout ea26b89aa302b1456c825fb622b414bb3b1013ce
```
Build the libfranka using the following commands:
```
cd ~/franka_research_ws/src/libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```
to test libfranka installation, please go to examples

```
cd ~/franka_research_ws/src/libfranka/build/
```

and run 
```
./communication_test <robot_ip>
```

*Troubleshooting for libfranka:*
1. In case libfranka is giving mismatch or unknown error, just build libfranka outside of the franka_research_ws using a method given in documentation.

```
cd ~
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
cd ~/libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```
2. If any of the examples in libfranka is asking for sudo permission, then add the current usergroup for rt permission as shown in:

https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel


## franka_ros