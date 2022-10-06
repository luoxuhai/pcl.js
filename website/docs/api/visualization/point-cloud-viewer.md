# PointCloudViewer

Simple point cloud visualization class.

Install Dependencies:

```bash
yarn add three
or 
npm install three --save
```

:::caution

The version of Three.js needs to be greater than 0.123.0

:::

For Example:

```ts
import PointCloudViewer from 'pcl.js/PointCloudViewer'

const pointCloudViewer = new PointCloudViewer(document.getElementById('canvas'), 500, 500);
pointCloudViewer.addPointCloud(cloud)
```

![PointCloudViewer](/img/examples/PointCloudViewer.png)

## Constructor

```ts
new PointCloudViewer(canvasEl, width, height);
```

**Parameters:**

| Name     | Type     | Default | Description                                                                                                       |
| -------- | -------- | ------- | ----------------------------------------------------------------------------------------------------------------- |
| canvasEl | `DOM`    |         | A [canvas](https://developer.mozilla.org/en-US/docs/Web/HTML/Element/canvas) where the renderer draws its output. |
| width    | `number` | 200     | The canvas width                                                                                                  |
| height   | `number` | 200     | The canvas height                                                                                                 |

## Methods

### addPointCloud()

```ts
addPointCloud(cloud, id);
```

**Parameters:**

| Name  | Type                                                | Default       | Description            |
| ----- | --------------------------------------------------- | ------------- | ---------------------- |
| cloud | [PointCloud](/docs/api/basic-structures#pointcloud) |               | The point cloud data . |
| id    | `string`                                            | 'point-cloud' | The point cloud id     |

### addPointCloudByUrl()

```ts
addPointCloudByUrl(url, id, onProgress)
```

### removePointCloud()

```ts
removePointCloud(id);
```

### setPointCloudProperties()

```ts
setPointCloudProperties(properties);
```

### setBackgroundColor()

```ts
setBackgroundColor(color);
```

### setCameraParameters()

```ts
setCameraParameters(parameters);
```

### setOrbitControls()

```ts
setOrbitControls(properties);
```

### setAxesHelper()

```ts
setAxesHelper(properties);
```

### setGridHelper()

```ts
setGridHelper(properties);
```

### setSize()

```ts
setSize(width, height);
```

### show()

```ts
show();
```

### hidden()

```ts
hidden();
```

### destroy()

```ts
destroy();
```
