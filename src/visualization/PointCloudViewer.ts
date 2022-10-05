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

interface CloudProperties {
  sizeAttenuation: boolean;
  size: number;
  color: ColorRepresentation;
  id: string;
}

type PointsObject3D = Points<BufferGeometry, PointsMaterial>;

class PointCloudViewer {
  public scene = new Scene();

  private camera = new PerspectiveCamera();
  private renderer: WebGLRenderer;
  private axesHelper?: AxesHelper;
  private gridHelper?: GridHelper;
  private controls: OrbitControls;
  private clouds: PointsObject3D[] = [];
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

  public addPointCloud(cloud: PointCloud, id?: string) {
    this.removePointCloud(id);

    const position: number[] = [];
    const { points } = cloud;
    const size = points.size;
    for (let i = 0; i < size; i++) {
      const point = points.get(i);
      position.push(point.x!, point.y!, point.z!);
    }

    const geometry = new BufferGeometry();
    const material = new PointsMaterial();

    if (position.length) {
      geometry.setAttribute('position', new Float32BufferAttribute(position, 3));
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

  public setPointCloudProperties(properties?: Partial<CloudProperties>, id?: string) {
    this.cloudProperties = { ...this.cloudProperties, ...properties };

    if (!this.clouds.length) {
      return;
    }

    const setProperties = (cloud: PointsObject3D) => {
      cloud.material.sizeAttenuation = this.cloudProperties.sizeAttenuation;
      cloud.material.size = this.cloudProperties.size;
      cloud.material.color.set(new Color(this.cloudProperties.color));
      cloud.name = this.cloudProperties.id;
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
    this.camera.fov = properties.fov;
    this.camera.aspect = properties.aspect;
    this.camera.near = properties.near;
    this.camera.far = properties.far;
    if (properties.position) {
      const { x, y, z } = properties.position;
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
      this.controls[key] = properties[key] ?? true;
    }
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
    this.scene.add(cloud);
    this.clouds.push(cloud);
    this.setPointCloudProperties({ id });
    this.setOrbitControls({ target: getCenter(cloud.geometry) });
  }
}

export default PointCloudViewer;
