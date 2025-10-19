import { file } from "bun";

export async function parseMap(
  mapFilePath: string
): Promise<[string[], number, number]> {
  const text = await file(mapFilePath).text();
  const lines = text
    .split("\n")
    .map((line) => line.trim())
    .filter(Boolean);

  const height = +lines[1]!.split(/\s+/)[1]!;
  const width = +lines[2]!.split(/\s+/)[1]!;

  const mapData = lines.slice(4).map((line) => line.trim());

  return [mapData, width, height];
}

export async function parseScen(
  filePath: string
): Promise<[[string, string][], number]> {
  const text = await file(filePath).text();
  const lines = text
    .split("\n")
    .map((line) => line.trim())
    .filter(Boolean);

  const data = lines.slice(1).map((line) => {
    const columns = line.split("\t");
    const startX = columns[4];
    const startY = columns[5];
    return [startX, startY] as [string, string];
  });

  const numAgents = data.length;

  return [data, numAgents];
}
