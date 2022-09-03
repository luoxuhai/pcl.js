/**
 * readPCDHeader
 *
 * https://github.com/mrdoob/three.js/blob/master/examples/jsm/loaders/PCDLoader.js
 * @param buffer
 * @returns
 */
export function readPCDHeader(buffer: ArrayBuffer) {
  const data = new TextDecoder().decode(new Uint8Array(buffer));
  const PCDheader: {
    data: any;
    str: any;
    headerLen: any;
    version: any;
    fields: any;
    size: any;
    type: any;
    count: any;
    width: any;
    height: any;
    viewpoint: any;
    points: any;
    offset: any;
    rowSize: any;
  } = {};
  const result1 = data.search(/[\r\n]DATA\s(\S*)\s/i);
  const result2 = /[\r\n]DATA\s(\S*)\s/i.exec(data.slice(result1 - 1));

  if (!result2) {
    return;
  }

  PCDheader.data = result2[1];
  PCDheader.headerLen = result2[0].length + result1;
  PCDheader.str = data.slice(0, PCDheader.headerLen);

  // remove comments

  // eslint-disable-next-line no-useless-escape
  PCDheader.str = PCDheader.str.replace(/\#.*/gi, '');

  // parse

  PCDheader.version = /VERSION (.*)/i.exec(PCDheader.str);
  PCDheader.fields = /FIELDS (.*)/i.exec(PCDheader.str);
  PCDheader.size = /SIZE (.*)/i.exec(PCDheader.str);
  PCDheader.type = /TYPE (.*)/i.exec(PCDheader.str);
  PCDheader.count = /COUNT (.*)/i.exec(PCDheader.str);
  PCDheader.width = /WIDTH (.*)/i.exec(PCDheader.str);
  PCDheader.height = /HEIGHT (.*)/i.exec(PCDheader.str);
  PCDheader.viewpoint = /VIEWPOINT (.*)/i.exec(PCDheader.str);
  PCDheader.points = /POINTS (.*)/i.exec(PCDheader.str);

  // evaluate

  if (PCDheader.version !== null)
    PCDheader.version = parseFloat(PCDheader.version[1]);

  PCDheader.fields =
    PCDheader.fields !== null ? PCDheader.fields[1].split(' ') : [];

  if (PCDheader.type !== null) PCDheader.type = PCDheader.type[1].split(' ');

  if (PCDheader.width !== null) PCDheader.width = parseInt(PCDheader.width[1]);

  if (PCDheader.height !== null)
    PCDheader.height = parseInt(PCDheader.height[1]);

  if (PCDheader.viewpoint !== null)
    PCDheader.viewpoint = PCDheader.viewpoint[1];

  if (PCDheader.points !== null)
    PCDheader.points = parseInt(PCDheader.points[1], 10);

  if (PCDheader.points === null)
    PCDheader.points = PCDheader.width * PCDheader.height;

  if (PCDheader.size !== null) {
    PCDheader.size = PCDheader.size[1].split(' ').map(function (x) {
      return parseInt(x, 10);
    });
  }

  if (PCDheader.count !== null) {
    PCDheader.count = PCDheader.count[1].split(' ').map(function (x) {
      return parseInt(x, 10);
    });
  } else {
    PCDheader.count = [];

    for (let i = 0, l = PCDheader.fields.length; i < l; i++) {
      PCDheader.count.push(1);
    }
  }

  PCDheader.offset = {};

  let sizeSum = 0;

  for (let i = 0, l = PCDheader.fields.length; i < l; i++) {
    if (PCDheader.data === 'ascii') {
      PCDheader.offset[PCDheader.fields[i]] = i;
    } else {
      PCDheader.offset[PCDheader.fields[i]] = sizeSum;
      sizeSum += PCDheader.size[i] * PCDheader.count[i];
    }
  }

  // for binary only

  PCDheader.rowSize = sizeSum;

  return PCDheader;
}
