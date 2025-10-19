// @jsxImportSource jsx-xml

import { render } from "jsx-xml";

type ConfigProps = {
  mapData: string[];
  width: number;
  height: number;
  robotInitPos: [string, string][];
  currNumAgent: number;
  portNum: number;
  visualization?: boolean;
  acceleration?: number;
  maxSpeed?: number;
};

export function Config({
  mapData,
  width,
  height,
  robotInitPos,
  currNumAgent,
  portNum,
  visualization = false,
  acceleration = 10,
  maxSpeed = 500,
}: ConfigProps) {
  const obstacles = ["@", "T"];
  const mapCenterX = -height / 2 + 0.5;
  const mapCenterY = -width / 2 + 0.5;
  return (
    <argos-configuration>
      <framework>
        <system threads="32" />
        <experiment
          length="0"
          ticks_per_second="10"
          random_seed="124"
          {...(!visualization && { visualization: "none" })}
        />
      </framework>
      <controllers>
        <footbot_diffusion_controller
          id="fdc"
          library="build/controllers/footbot_diffusion/libfootbot_diffusion"
        >
          <actuators>
            <differential_steering implementation="default" />
          </actuators>
          <sensors>
            <footbot_proximity implementation="default" show_rays="true" />
            <positioning implementation="default" />
          </sensors>
          <params
            alpha="7.5"
            omega="3.0"
            velocity={`${maxSpeed}`}
            acceleration={`${acceleration}`}
            portNumber={`${portNum}`}
            outputDir={`metaData${portNum}/`}
          />
        </footbot_diffusion_controller>
      </controllers>
      <arena
        size={`${height},${width},1`}
        center={`${mapCenterX},${mapCenterY},0`}
      >
        {mapData.map((row, y) =>
          row.split("").map((cell, x) =>
            obstacles.includes(cell) ? (
              <box
                id={`box_${x}_${y}`}
                size="0.9,0.9,0.1"
                movable="false"
                key={`box_${x}_${y}`}
              >
                <body position={`${-y},${-x},0`} orientation="0,0,0" />
              </box>
            ) : null
          )
        )}
        {robotInitPos.slice(0, currNumAgent).map(([x, y], i) => (
          <foot-bot id={`fb_${x}_${y}`} index={`${i}`} key={`fb_${x}_${y}`}>
            <body
              position={`${-Number(y)},${-Number(x)},0`}
              orientation="0,0,0"
            />
            <controller config="fdc" />
          </foot-bot>
        ))}
      </arena>
      <physics_engines>
        <dynamics2d id="dyn2d" />
      </physics_engines>
      <media />
      {visualization && (
        <visualization>
          <external_visualizer>
            <camera>
              <placements>
                <placement
                  index="0"
                  position={`${mapCenterX},${mapCenterY},${Math.max(
                    width,
                    height
                  )}`}
                  look_at={`${mapCenterX},${mapCenterY},0`}
                  up="1,0,0"
                  lens_focal_length="20"
                />
              </placements>
            </camera>
          </external_visualizer>
        </visualization>
      )}
    </argos-configuration>
  );
}

export function createArgosConfig(props: ConfigProps) {
  return render(<Config {...props} />, {
    version: "1.0",
  }).end({ prettyPrint: true });
}
