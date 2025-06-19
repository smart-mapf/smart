![Banner](https://github.com/user-attachments/assets/c43d8b2d-16e2-45c4-999e-b25d45b9cb1b)

[Open the app](https://smart-mapf.github.io/demo/)

---

# Scalable Multi-Agent Realistic Testbed (SMART)

SMART is an efficient soft-tool that bridge the gap between Multi-Agent Path Finding (MAPF) and real-world application. More details can be found in our paper:
Advancing MAPF towards the Real World: A Scalable Multi-Agent Realistic Testbed (SMART) [1].


## Requirements

### Ubuntu 22.04

### RPC
This repo requires [RPC](https://github.com/rpclib/rpclib) for communication
between server and clients.
It also requires [Planner](https://github.com/lunjohnzhang/MAPF-LNS2) for planning.
Please install rpc using:
```angular2html
cd client
mkdir externalDependencies
cd externalDependencies
git clone --recurse-submodules https://github.com/rpclib/rpclib
cd rpclib
mkdir build
cd build
cmake ..
make
sudo make install
```

### Argos
Install Argos 3, please refer to this [Link](https://www.argos-sim.info/core.php) for instruction.


## Compile instructions
### Client Folder (Simulator)
- To build the project, first navigate to the client folder Then: 
```
cd ./client
mkdir build
cd build
```

- To produce debuggable code (slow), type:

```angular2html
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
cd ..
```

If a linking error occurs, please remove the conda from your path (e.g. in your ~/.bashrc).

To produce fast but not debuggable code, type:
```angular2html
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..
```

- You can verify the correctness of the compilation by running: 
```angular2html
argos3 -c experiments/diffusion_1.argos
```

### Server Folder (ADG)
- Compile the code:
```angular2html
cd ./server
mkdir build
cd build
cmake ..
make
```

## Running guide for entire pipeline

### Running with visualization
```
python run_sim.py --map_name=random-32-32-20.map --scen_name=random-32-32-20-random-1.scen --num_agents=50 --path_filename=example_paths_xy.txt --flip_coord=0
```
```
python run_sim.py --map_name=random-32-32-20.map --scen_name=random-32-32-20-random-1.scen --num_agents=50 --path_filename=example_paths_yx.txt --flip_coord=1
```

### Running in headless mode
```
python run_sim.py --map_name=random-32-32-20.map --scen_name=random-32-32-20-random-1.scen --num_agents=50 --path_filename=example_paths_xy.txt --flip_coord=0 --headless=True
```
```
python run_sim.py --map_name=random-32-32-20.map --scen_name=random-32-32-20-random-1.scen --num_agents=50 --path_filename=example_paths_yx.txt --flip_coord=1 --headless=True
```


## Reference
[1] Yan, Jingtian, Zhifei Li, William Kang, Kevin Zheng, Yulun Zhang, Zhe Chen, Yue Zhang, Daniel Harabor, Stephen Smith, and Jiaoyang Li. "Advancing MAPF towards the Real World: A Scalable Multi-Agent Realistic Testbed (SMART)." arXiv preprint arXiv:2503.04798 (2025).
