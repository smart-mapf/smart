import { spawn, sleep, file } from "bun";
import fs from "fs";
import path from "path";

// Replace these imports with your own JS/TS equivalents of ArgosConfig functions
import { createArgosConfig } from "./createArgosConfig";
import { parseMap, parseScen } from "./parse";

function checkFile(filePath: string): boolean {
  if (!fs.existsSync(filePath)) {
    console.log(`${filePath} does not exist!`);
    return false;
  }
  return true;
}

async function runExperiment(serverCommand: string[], clientCommand: string[]) {
  const serverProcess = spawn(serverCommand);
  await sleep(1000);
  const clientProcess = spawn(clientCommand, {
    cwd: path.resolve(process.cwd(), "client"),
  });
  return { serverProcess, clientProcess };
}

const SERVER_EXECUTABLE_PATH = "server/build/ADG_server";

  
type Options = {
  map_name: string;
  scen_name: string;
  num_agents: number;
  headless?: boolean;
  argos_config_name?: string;
  path_filename?: string;
  stats_name?: string;
  port_num?: number;
  flip_coord?: number;
  max_speed?: number;
  acceleration?: number;
};

export async function run({
    map_name,
    scen_name,
    num_agents,
    headless = false,
    argos_config_name = "output.argos",
    path_filename = "example_paths.txt",
    stats_name = "stats.csv",
    port_num = 8182,
    flip_coord = 1,
    max_speed = 500,
    acceleration = 10,
  }: Options) {

  console.log(`Map Name: ${map_name}`);
  console.log(`Scenario Name: ${scen_name}`);

  if (!(await file(map_name).exists())) {
    throw new Error("Map file not found");
  }
  if (!(await file(scen_name).exists())) {
    throw new Error("Scenario file not found");
  }

  console.log("Creating Argos config file ...");
  const [robot_init_pos, scen_num_agent] = await parseScen(scen_name);
  const [map_data, width, height] = await parseMap(map_name);

  if (scen_num_agent < num_agents) {
    throw new Error("Number of agents exceed maximum number. exiting ...")
  }

  const c1 = createArgosConfig({
    mapData: map_data,
    width,
    height,
    robotInitPos: robot_init_pos,
    currNumAgent: num_agents,
    portNum: port_num,
    visualization: !headless,
    maxSpeed: max_speed,
    acceleration: acceleration,
  });
  console.log("Argos config file created.");

  console.log("Running simulator ...");
  const serverCommand = [
    SERVER_EXECUTABLE_PATH,
    "-p",
    path_filename,
    "-n",
    String(port_num),
    "-o",
    stats_name,
    "-m",
    map_name,
    "-s",
    String(scen_name),
    `--method_name=LNS2`,
    `--flip_coord=${flip_coord}`,
  ];
  console.log(serverCommand);

  const clientCommand = ["argos3", "-c", `../${argos_config_name}`];
  console.log(clientCommand);

  return await runExperiment(serverCommand, clientCommand);
}
