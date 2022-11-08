import Table from "cli-table";
import fs from "fs";
import yaml from "js-yaml";
import convert from "xml-js";

import { PortInfo } from "./lego/types";
import { configuration } from "./configuration";

export const printDevicesOverview = (devicesOnPorts: PortInfo[]) => {
  console.log("Overview of used devices and ports:");

  const table = new Table({ head: ["Type", "Port", "Color"] });

  const rows = devicesOnPorts.map(({ name, type, color }) => [type, name, color]);

  table.push(...rows);
  console.log(table.toString());
};

export const writeDeviceInfoXML = (devicesOnPorts: PortInfo[], robotName: string) => {
  const declaration = {
    _declaration: {
      _attributes: {
        version: "1.0",
        encoding: "utf-8"
      }
    }
  };

  const devices = [];

  for (const d of devicesOnPorts) {
    const { type, name } = d;

    const pluginName = configuration.plugins[type];

    if (!pluginName) {
      console.log("No plugin available for type", type);
      continue;
    }

    const pluginElement = {
      _attributes: {
        type: pluginName
      },
      robotName: {
        _text: robotName
      },
      port: {
        _text: name
      }
    };

    devices.push(pluginElement);
  }

  const completeXmlDescription = {
    ...declaration,
    robot: {
      _attributes: {
        name: robotName
      },
      webots: {
        plugin: [
          ...devices,
          {
            _attributes: {
              type: configuration.plugins.clock
            },
            robotName: {
              _text: robotName
            }
          }
        ]
      }
    }
  };

  const xml = convert.json2xml(JSON.stringify(completeXmlDescription), {
    spaces: 4,
    compact: true
  });

  fs.writeFileSync(configuration.directories.robotConfiguration + "/" + robotName + ".urdf", xml);
};

export const getSensorConfig = (index: number) =>
  configuration.sensorsOnPorts[index % configuration.sensorsOnPorts.length];
