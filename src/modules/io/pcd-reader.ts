interface PCDHeader {
  version: number;
  fields: string[];
  type: ('F' | 'I' | 'U')[];
  count: number[];
  size: number[];
  offset: number[];
  width: number;
  height: number;
  points: number;
  viewpoint: string;
  data: 'ascii' | 'binary' | 'binary_compressed';
}

/**
 * {@link https://github.com/mrdoob/three.js/blob/master/examples/jsm/loaders/PCDLoader.js}
 * {@link https://pointclouds.org/documentation/tutorials/pcd_file_format.html}
 *
 * @param buffer pcd file data
 * @returns pcd header
 */
function readPCDHeader(buffer: ArrayBuffer) {
  const data = new TextDecoder().decode(new Uint8Array(buffer));
  const PCDHeader: PCDHeader = {
    data: 'ascii',
    version: 0.7,
    fields: [],
    size: [],
    type: [],
    count: [],
    width: 0,
    height: 0,
    viewpoint: '',
    points: 0,
    offset: [],
  };
  const result1 = data.search(/[\r\n]DATA\s(\S*)\s/i);
  const result2 = /[\r\n]DATA\s(\S*)\s/i.exec(data.slice(result1 - 1));

  if (!result2) {
    return;
  }

  PCDHeader.data = result2[1] as PCDHeader['data'];
  const headerLen = result2[0].length + result1;
  const str = data.slice(0, headerLen).replace(/#.*/gi, '');

  // parse

  const version = /VERSION (.*)/i.exec(str);
  const fields = /FIELDS (.*)/i.exec(str);
  const size = /SIZE (.*)/i.exec(str);
  const type = /TYPE (.*)/i.exec(str);
  const count = /COUNT (.*)/i.exec(str);
  const width = /WIDTH (.*)/i.exec(str);
  const height = /HEIGHT (.*)/i.exec(str);
  const viewpoint = /VIEWPOINT (.*)/i.exec(str);
  const points = /POINTS (.*)/i.exec(str);

  // evaluate

  if (version !== null) PCDHeader.version = parseFloat(version[1]);

  if (fields !== null) PCDHeader.fields = fields[1].split(' ');

  if (type !== null) PCDHeader.type = type[1].split(' ') as PCDHeader['type'];

  if (width !== null) PCDHeader.width = parseInt(width[1]);

  if (height !== null) PCDHeader.height = parseInt(height[1]);

  if (viewpoint !== null) PCDHeader.viewpoint = viewpoint[1];

  if (points !== null) PCDHeader.points = parseInt(points[1], 10);

  if (points === null) PCDHeader.points = PCDHeader.width * PCDHeader.height;

  if (size !== null) {
    PCDHeader.size = size[1].split(' ').map((x) => parseInt(x, 10));
  }

  if (count !== null) {
    PCDHeader.count = count[1].split(' ').map(function (x) {
      return parseInt(x, 10);
    });
  } else {
    PCDHeader.count = [];

    for (let i = 0, l = PCDHeader.fields.length; i < l; i++) {
      PCDHeader.count.push(1);
    }
  }

  let sizeSum = 0;

  for (let i = 0, l = PCDHeader.fields.length; i < l; i++) {
    if (PCDHeader.data === 'ascii') {
      PCDHeader.offset.push(i);
    } else {
      PCDHeader.offset.push(sizeSum);
      sizeSum += PCDHeader.size[i] * PCDHeader.count[i];
    }
  }

  return PCDHeader;
}

export { PCDHeader, readPCDHeader };
