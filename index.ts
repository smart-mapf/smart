import { file, readableStreamToText, spawn, write } from "bun";
import { getPort } from "get-port-please";
import { load } from "js-yaml";
import { has, isObject, isPlainObject } from "lodash";
import { temporaryFile } from "tempy";

type Options = {
  /**
   * The map used for the simulation. This is a string in the format
   * specified by the Argos simulator.
   */
  map: string;
  /**
   * The scenario used for the simulation. This is a string in the format
   * specified by the Argos simulator.
   */
  scen: string;
  /**
   * The paths used for the simulation. This is a string in the format
   * specified by the Argos simulator.
   */
  paths: string;
  /**
   * The number of agents used in the simulation.
   */
  agents: number;
  flipXY: boolean;
};

export type Step = {
  type: "tick";
  clock: number;
  agents: {
    id: number;
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  }[];
};

export type AdgProgress = {
  type: "adg_progress";
  constraints: {
    constraining_agent: {
      id: number;
    }[];
  }[];
};

export type Output =
  | Step
  | { type: "error"; error: any }
  | { type: "message"; content: string }
  | AdgProgress
  | Record<never, never>;

async function* streamLines(stream: AsyncIterableIterator<Uint8Array>) {
  let leftover = "";

  for await (const c of stream) {
    const decoder = new TextDecoder();
    const chunk = decoder.decode(c);
    // Ensure we are working with string data
    const text = leftover + chunk.toString();
    const lines = text.split(/\r?\n/);
    leftover = lines.pop()!; // Save the last (possibly incomplete) line

    for (const line of lines) {
      yield line;
    }
  }

  if (leftover) {
    yield leftover; // Yield any remaining text after stream ends
  }
}

async function* buffered<T>(a: AsyncIterableIterator<T>, size = 64) {
  let buffer: T[] = [];
  for await (const item of a) {
    buffer.push(item);
    if (buffer.length >= size) {
      yield buffer;
      buffer = [];
    }
  }
  yield buffer;
}

function isOutput(a: unknown): a is Output {
  return isPlainObject(a) && has(a, "type");
}

/**
 * This function runs a simulation using the provided map, scenario, paths, and number of agents.
 * It creates temporary files for the map and paths, runs the simulation using a Python script,
 * and cleans up the temporary files after the simulation is complete.
 */
export async function run({ map, paths, agents, scen, flipXY }: Options) {
  const tmp = {
    map: temporaryFile({ extension: "map" }),
    scen: temporaryFile({ extension: "scen" }),
    paths: temporaryFile({ extension: "txt" }),
  };

  await write(tmp.map, map);
  await write(tmp.paths, paths);
  await write(tmp.scen, scen);

  const out = spawn(
    [
      "python",
      "run_sim.py",
      `--map_name=${tmp.map}`,
      `--scen_name=${tmp.scen}`,
      `--num_agents=${agents}`,
      `--path_filename=${tmp.paths}`,
      `--port=${await getPort()}`,
      `--flip_coord=${flipXY ? "1" : "0"}`,
    ],
    {
      stderr: "pipe",
      cwd: import.meta.dir,
      env: {
        ...process.env,
        ARGOS_PLUGIN_PATH: `${
          import.meta.dir
        }/plugins/visualizers/external_visualizer/build`,
      },
    }
  );

  const errors = readableStreamToText(out.stderr);

  return {
    errors,
    async *values() {
      async function* f(): AsyncGenerator<Output> {
        let ticked = false;
        for await (const line of streamLines(out.stdout.values())) {
          try {
            const out = load(line);
            if (isOutput(out) && "type" in out) {
              switch (out.type) {
                case "tick":
                  yield out;
                  ticked = true;
                  break;
                case "adg_progress":
                  if (ticked) {
                    yield out;
                    ticked = false;
                  }
                  break;
                default:
                  // Ignore other event types for now
                  break;
              }
            }
            // Do nothing for now if output is parsable but not an output
          } catch (e) {
            console.log(line);
            yield { type: "message", content: line };
            // Ignore parse errors
          }
        }
      }
      yield* buffered(f());
    },
    async dispose() {
      await file(tmp.map).delete();
      await file(tmp.paths).delete();
      await file(tmp.scen).delete();
      out.kill();
      console.warn("Stopped");
    },
  };
}
