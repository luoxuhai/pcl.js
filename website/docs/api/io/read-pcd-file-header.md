---
sidebar_position: 2
---

# readPCDFileHeader

```ts
readPCDFileHeader(filename)
```

Read the header of any PCD file.

**Parameters:**

| Name     | Type     | Default | Description                   |
| -------- | -------- | ------- | ----------------------------- |
| filename | `string` |         | The name of the file to load. |

**Returns:**

| Name   | Type                         |
| ------ | ---------------------------- |
| header | [`PCDHeader`](#pcdheader) |


## Type Definitions

### PCDHeader

```ts
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
```
