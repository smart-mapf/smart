// src/jsx.d.ts

import { JSX as JSXBase } from "jsx-xml";
import "react";

declare module "react" {
  namespace JSX {
    interface IntrinsicElements {
      body: any;
    }
  }
}
declare global {
  namespace JSX {
    interface IntrinsicElements extends JSXBase.IntrinsicElements {
      "argos-configuration"?: any;
      framework?: any;
      system?: any;
      experiment?: any;
      controllers?: any;
      footbot_diffusion_controller?: any;
      actuators?: any;
      differential_steering?: any;
      sensors?: any;
      footbot_proximity?: any;
      positioning?: any;
      params?: any;
      arena?: any;
      box?: any;
      body?: any;
      controller?: any;
      "foot-bot"?: any;
      physics_engines?: any;
      dynamics2d?: any;
      media?: any;
      visualization?: any;
      external_visualizer?: any;
      camera?: any;
      placements?: any;
      placement?: any;
    }
  }
}
