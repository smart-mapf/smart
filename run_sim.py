import os
import argparse
import ArgosConfig
import subprocess
import time
import xml.etree.ElementTree as ET

def parse_arguments():
    parser = argparse.ArgumentParser(description="Argument parser for map_name and scen_name.")
    parser.add_argument("--map_name", type=str, required=True, help="Name of the map file")
    parser.add_argument("--scen_name", type=str, required=True, help="Name of the scenario file")
    parser.add_argument("--num_agents", type=int, required=True, help="Number of agents in the scenario")
    parser.add_argument("--headless", type=bool, required=False, default=False, help="Simulator run in headless mode")
    parser.add_argument("--argos_config_name", type=str, required=False, default="output.argos", help="Name of the argos config file")
    parser.add_argument("--path_filename", type=str, required=False, default="example_paths.txt", help="Name of the output path file")
    parser.add_argument("--stats_name", type=str, required=False, default="stats.csv", help="Name of the statistics file for simulator")
    parser.add_argument("--port_num", type=int, required=False, default=8182, help="Port number for sim and client")
    parser.add_argument("--flip_coord", type=int, required=False, default=True, help="input format of the mapf planner, 0 if xy, 1 if yx")

    return parser.parse_args()


def run_planner(command: list):
    
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)

        if result.stderr:
            print("Errors:")
            print(result.stderr)
            return 0

        if "Succeed" in result.stdout:
            return 1
        else:
            if "Fail" in result.stdout or "Timeout" in result.stdout:
                return 0
            else:
                return 1

    except subprocess.CalledProcessError as e:
        print(f"Execution failed: {e}")
        print(f"Error output: {e.stderr}")
        return 0

def check_file(file_path:str):
    if not os.path.exists(file_path):
        print(f"{file_path} not exists!")
        return False
    return True


def run_experiment(args):
    server_command, client_command = args
    server_process = subprocess.Popen(server_command)

    time.sleep(1)
    # os.chdir("client")

    client_process = subprocess.Popen(client_command)

    client_process.wait()
    # os.chdir("..")
    server_process.wait()


def get_sim_dt_from_argos_config(config_filename: str) -> float:
    default_dt = 0.1
    try:
        tree = ET.parse(config_filename)
        root = tree.getroot()
        experiment = root.find("./framework/experiment")
        if experiment is None:
            return default_dt
        ticks_per_second = float(experiment.attrib.get("ticks_per_second", "10"))
        if ticks_per_second <= 0:
            return default_dt
        return 1.0 / ticks_per_second
    except Exception:
        return default_dt

if __name__ == "__main__":
    args = parse_arguments()
    print(f"Map Name: {args.map_name}")
    print(f"Scenario Name: {args.scen_name}")


    scen_file_path = args.scen_name
    map_file_path = args.map_name
    config_filename = args.argos_config_name
    curr_num_agent = args.num_agents
    port_num=args.port_num
    path_filename = args.path_filename
    sim_stats_filename = args.stats_name

    if not check_file(map_file_path):
        print("Map file not exists!")
        exit(-1)
    if not check_file(scen_file_path):
        print("Scenario file not exists!")
        exit(-1)

    print("Creating Argos config file ...")
    robot_init_pos, scen_num_agent = ArgosConfig.read_scen(scen_file_path)
    map_data, width, height = ArgosConfig.parse_map_file(map_file_path)
    if scen_num_agent < curr_num_agent:
        print("Number of agents exceed maximum number. exiting ...")
        exit(-1)
    ArgosConfig.create_Argos(map_data, config_filename, width, height, robot_init_pos, curr_num_agent, port_num, not args.headless)
    print("Argos config file created.")

    sim_dt = get_sim_dt_from_argos_config(config_filename)
    print(f"Simulation dt: {sim_dt} seconds")

    print("Running simulator ...")
    server_executable_path = "build/server/ADG_server"
    server_command = [server_executable_path, "-p", path_filename, "-n", str(port_num), "-o",
                      sim_stats_filename, "-m", map_file_path, "-s", str(scen_file_path), f"--method_name=LNS2",
                      f"--flip_coord={args.flip_coord}", f"--sim_dt={sim_dt}"]
    print(server_command)
    client_command = ["argos3", "-c", f"./{config_filename}"]
    print(client_command)
    # exit(0)
    run_experiment((server_command, client_command))
