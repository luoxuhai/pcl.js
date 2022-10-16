import {
  Scene,
  WebGLRenderer,
  PerspectiveCamera,
  AxesHelper,
  GridHelper,
  Points,
  PointsMaterial,
  ColorRepresentation,
  BufferGeometry,
  Color,
  Vector3,
  Float32BufferAttribute,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { PCDLoader } from 'three/examples/jsm/loaders/PCDLoader';

import { getCenter } from './utils';
import { PointCloud } from '@/modules/common/PointCloud';
import { XYZPointTypes, RGBPointTypes } from '@/modules/common/point-types';

interface CloudProperties {
  sizeAttenuation: boolean;
  size: number;
  color: ColorRepresentation;
  id: string;
}

type PointsObject3D = Points<BufferGeometry, PointsMaterial>;

class PointCloudViewer {
  public scene = new Scene();
  public clouds: PointsObject3D[] = [];

  private camera = new PerspectiveCamera();
  private renderer: WebGLRenderer;
  private axesHelper?: AxesHelper;
  private gridHelper?: GridHelper;
  private controls: OrbitControls;
  private cloudProperties: CloudProperties = {
    sizeAttenuation: false,
    size: 1,
    color: '#FFF',
    id: 'point-cloud',
  };

  constructor(canvas: HTMLCanvasElement, width = 200, height = 200) {
    this.renderer = new WebGLRenderer({
      powerPreference: 'high-performance',
      canvas,
      antialias: true,
    });
    this.controls = this.createOrbitControls();
    const near = 0.0001;
    const far = 10000;
    this.setOrbitControls({ minDistance: near, maxDistance: far });
    this.setCameraParameters({
      fov: 45,
      aspect: width / height,
      near,
      far,
      position: {
        x: 0,
        y: 0,
        z: 10,
      },
    });
    this.setSize(width, height);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setAnimationLoop(this.render);
  }

  // public function

  public addPointCloud(cloud: PointCloud<XYZPointTypes>, id?: string) {
    this.removePointCloud(id);

    const position: number[] = [];
    const color: number[] = [];
    const { points } = cloud;
    const size = points.size;
    for (let i = 0; i < size; i++) {
      const point = points.get(i) as RGBPointTypes;
      position.push(point.x, point.y, point.z);
      if (point.r !== undefined) {
        color.push(point.r, point.g, point.b);
      }
    }

    const geometry = new BufferGeometry();
    const material = new PointsMaterial();

    if (position.length) {
      geometry.setAttribute('position', new Float32BufferAttribute(position, 3));
    }

    if (color.length) {
      geometry.setAttribute('color', new Float32BufferAttribute(color, 3));
      material.vertexColors = true;
      material.needsUpdate = true;
    }

    this.addPointCloudToScene(new Points(geometry, material), id);
  }

  public async addPointCloudByUrl(
    url: string,
    id?: string,
    onProgress?: ((event: ProgressEvent<EventTarget>) => void) | undefined,
  ) {
    this.removePointCloud();
    const cloud = (await new PCDLoader().loadAsync(url, onProgress)) as PointsObject3D;
    if (cloud) {
      this.addPointCloudToScene(cloud, id);
    }
  }

  public async addPointCloudByData(data: ArrayBuffer, id?: string) {
    const cloud = new PCDLoader().parse(data, '') as PointsObject3D;
    this.addPointCloudToScene(cloud, id);
  }

  public removePointCloud(id = this.cloudProperties.id) {
    const object = this.scene.getObjectByName(id) as PointsObject3D;
    object?.material?.dispose();
    object?.geometry?.dispose();
    object?.removeFromParent();
    const index = this.clouds.findIndex((v) => v.name === id);
    this.clouds.splice(index, 1);
  }

  public setPointCloudProperties(properties?: Partial<CloudProperties>) {
    this.cloudProperties = { ...this.cloudProperties, ...properties };
    const { id } = this.cloudProperties;

    if (!this.clouds.length) {
      return;
    }

    const setProperties = (cloud: PointsObject3D) => {
      cloud.material.sizeAttenuation = this.cloudProperties.sizeAttenuation;
      cloud.material.size = this.cloudProperties.size;
      cloud.material.color.set(new Color(this.cloudProperties.color));
    };

    if (id) {
      const object = this.clouds.find((v) => v.name === id);
      if (object) {
        setProperties(object);
      }
    } else {
      this.clouds.forEach(setProperties);
    }
  }

  public setBackgroundColor(bgc: ColorRepresentation) {
    this.renderer.setClearColor(bgc);
  }

  public setCameraParameters(properties: {
    fov: number;
    aspect: number;
    near: number;
    far: number;
    position?: Pick<Vector3, 'x' | 'y' | 'z'>;
  }) {
    const { fov, aspect, near, far, position } = properties;
    this.camera.fov = fov ?? this.camera.fov;
    this.camera.aspect = aspect ?? this.camera.aspect;
    this.camera.near = near ?? this.camera.near;
    this.camera.far = far ?? this.camera.far;
    if (position) {
      const { x, y, z } = position;
      this.camera.position.set(x, y, z);
    }
  }

  public setOrbitControls(properties: {
    enableRotate?: boolean;
    enableZoom?: boolean;
    enablePan?: boolean;
    enableDamping?: boolean;
    enabled?: boolean;
    minDistance?: number;
    maxDistance?: number;
    target?: Pick<Vector3, 'x' | 'y' | 'z'>;
  }) {
    for (const key in properties) {
      if (properties[key] !== undefined) {
        this.controls[key] = properties[key];
      }
    }
    this.controls.update();
  }

  public setAxesHelper(properties: { visible: boolean; size?: number }) {
    const { visible = true, size = 10 } = properties ?? {};

    if (!this.axesHelper) {
      this.axesHelper = new AxesHelper(size);
      this.scene.add(this.axesHelper);
    }

    this.axesHelper.visible = visible;
  }

  public setGridHelper(properties: { visible: boolean; size?: number; divisions?: number }) {
    const { visible = true, size = 100, divisions = 100 } = properties ?? {};

    if (!this.gridHelper) {
      this.gridHelper = new GridHelper(size, divisions);
      this.scene.add(this.gridHelper);
    }

    this.gridHelper.visible = visible;
  }

  public setSize(width: number, height: number) {
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  public show() {
    this.scene.visible = true;
  }

  public hidden() {
    this.scene.visible = false;
  }

  public destroy() {
    this.renderer.dispose();
  }

  // private function

  private render = () => {
    this.renderer.render(this.scene, this.camera);
  };

  private createOrbitControls() {
    const controls = new OrbitControls(this.camera, this.renderer.domElement);

    return controls;
  }

  private addPointCloudToScene(cloud: PointsObject3D, id?: string) {
    cloud.name = id ?? this.cloudProperties.id;
    this.scene.add(cloud);
    this.clouds.push(cloud);
    this.setPointCloudProperties();
    this.setOrbitControls({ target: getCenter(cloud.geometry) });
  }
}

export default PointCloudViewer;
