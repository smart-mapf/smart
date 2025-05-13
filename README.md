# Scalable Multi-Agent Realistic Testbed (SMART)

### Output format for ADG progress

We return the current progress of ADG in the following format:

```yaml
{type: adg_progress, agent_id: 0, constraining_agent: [{id: 3, }, ], agent_id: 1, constraining_agent: [{id: 7, }, ], agent_id: 2, constraining_agent: [{id: 17, }, ], agent_id: 3, constraining_agent: [{id: 7, }, ], agent_id: 4, constraining_agent: [{id: 14, }, ], agent_id: 5, constraining_agent: [], agent_id: 6, constraining_agent: [], agent_id: 7, constraining_agent: [{id: 4, }, ], agent_id: 8, constraining_agent: [], agent_id: 9, constraining_agent: [], agent_id: 10, constraining_agent: [{id: 2, }, ], agent_id: 11, constraining_agent: [{id: 4, }, {id: 4, }, ], agent_id: 12, constraining_agent: [], agent_id: 13, constraining_agent: [{id: 19, }, ], agent_id: 14, constraining_agent: [{id: 6, }, {id: 6, }, ], agent_id: 15, constraining_agent: [{id: 9, }, ], agent_id: 16, constraining_agent: [{id: 5, }, ], agent_id: 17, constraining_agent: [], agent_id: 18, constraining_agent: [], agent_id: 19, constraining_agent: [], }
```

- type indicates the print type;
- agent_id is the id of the robot;
- constraining_agent includes a list of agents that block the last action of the agent_id;

### Requirements

#### Ubuntu 22.04

#### RPC

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

#### Argos

Install Argos 3, please refer to this [Link](https://www.argos-sim.info/core.php) for instruction.

### Compile instructions

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

### Running guide for entire pipeline

#### Running with visualization

```
python run_sim.py --map_name=random-32-32-20.map --scen_name=random-32-32-20-random-1.scen --num_agents=20 --path_filename=example_paths.txt
```

#### Running in headless mode

```
python run_sim.py --map_name=random-32-32-20.map --scen_name=random-32-32-20-random-1.scen --num_agents=20 --path_filename=example_paths.txt --headless=True
```
