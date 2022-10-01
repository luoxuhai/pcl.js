# Changelog

## [1.7.2](https://github.com/luoxuhai/pcl.js/compare/v1.7.1...v1.7.2) (2022-10-01)


### 🐛 Bug Fixes

* can't get PointXYZRGB to rgb value ([be97496](https://github.com/luoxuhai/pcl.js/commit/be974960bd4d84085b4a5db2e3bf4cb83b3abfb8))


### ✨ Features

* new readPCDHeader method ([2b56103](https://github.com/luoxuhai/pcl.js/commit/2b56103df15740f7a36b9a4144b334ebe931ba0d))

## [1.7.1](https://github.com/luoxuhai/pcl.js/compare/v1.7.0...v1.7.1) (2022-09-29)


### 🐛 Bug Fixes

* get points cause memory leak ([7bbc015](https://github.com/luoxuhai/pcl.js/commit/7bbc01546f963006bdfe1e2f856266cd05884c1c))
* points.size is not a function at PointCloudViewer.addPointCloud ([fea411f](https://github.com/luoxuhai/pcl.js/commit/fea411f42e424a0e18c38f25af6c82b09fda7ffe))

## [1.7.0](https://github.com/luoxuhai/pcl.js/compare/v1.6.0...v1.7.0) (2022-09-29)


### ✨ Features

* add addPointCloudByData method to PointCloudViewer ([5a80639](https://github.com/luoxuhai/pcl.js/commit/5a806397fd5ee29d0383a0240902882cdbce0283))
* add bigint support ([96cf62c](https://github.com/luoxuhai/pcl.js/commit/96cf62cc0bce4242a541effad74b98903c07c230))
* add header and size to PointCLoud ([00f0a37](https://github.com/luoxuhai/pcl.js/commit/00f0a372d9e2da1df5f1afe699a9e97d6403cec4))


### 🐛 Bug Fixes

* computeCloudResolution fails to process point clouds that are not of type PointXYZ ([f6f9548](https://github.com/luoxuhai/pcl.js/commit/f6f9548deb7362024ba27bbb35885cfedb2b8add))

## [1.6.0](https://github.com/luoxuhai/pcl.js/compare/v1.5.0...v1.6.0) (2022-09-25)


### ✨ Features

* add ISSKeypoint3D ([018adfc](https://github.com/luoxuhai/pcl.js/commit/018adfc59c7e39b68c184c032cb54b66f7f94024))
* add search.KdTree api ([bad2092](https://github.com/luoxuhai/pcl.js/commit/bad2092efa89da77e979195c5a67169a036ad251))

# [1.5.0](https://github.com/luoxuhai/pcl.js/compare/v1.4.1...v1.5.0) (2022-09-23)


### Features

* add new io apis ([cbfc1be](https://github.com/luoxuhai/pcl.js/commit/cbfc1bef12af49ca18605c6fdba3f6332f6fb22e))

## [1.4.1](https://github.com/luoxuhai/pcl.js/compare/v1.4.0...v1.4.1) (2022-09-21)


### Bug Fixes

* setSize method of PointCloudViewer does not scale correctly ([6781d94](https://github.com/luoxuhai/pcl.js/commit/6781d942f6f33e8c2ed8b89cbc55176636c91be0))

### Documentation

* update StatisticalOutlierRemoval example url ([7b0f6cb](https://github.com/luoxuhai/pcl.js/commit/7b0f6cb81767b6dc1447f5190e7e3e41e70081ac))
* remove example detail page ([7f02e57](https://github.com/luoxuhai/pcl.js/commit/7f02e57194b239be1d28cb7f9cdaa193b04998f2))

# [1.4.0](https://github.com/luoxuhai/pcl.js/compare/v1.3.0...v1.4.0) (2022-09-21)


### Features

* add CloudViewer to visualize point cloud ([338373a](https://github.com/luoxuhai/pcl.js/commit/338373a59f8b7ffd9df8f2d209bada6b95569a51))
* add KdTreeFLANN ([25d8d98](https://github.com/luoxuhai/pcl.js/commit/25d8d98962091f494a99cb25601c2f95e6ce08c0))

# [1.3.0](https://github.com/luoxuhai/pcl.js/compare/v1.2.0...v1.3.0) (2022-09-17)


### Features

* add ApproximateVoxelGrid filter ([09b9e7a](https://github.com/luoxuhai/pcl.js/commit/09b9e7a9c2c4514f1a986faee107c53be20dedc7))
* add GridMinimum filter ([5b0c32f](https://github.com/luoxuhai/pcl.js/commit/5b0c32f043918bff0516ba5bef976b4c02e7eec7))
* add LocalMaximum filter ([d3d2131](https://github.com/luoxuhai/pcl.js/commit/d3d213126a0ff30b06f9b7880c44b2551e0f8537))
* add RandomSample filter ([9c0cd41](https://github.com/luoxuhai/pcl.js/commit/9c0cd4192cc5f279c12d7662c2c30327a5b2232a))
* add UniformSampling filter ([d2cf028](https://github.com/luoxuhai/pcl.js/commit/d2cf0283dd8807726d2df242a7fb22405040d849))

# [1.2.0](https://github.com/luoxuhai/pcl.js/compare/v1.1.1...v1.2.0) (2022-09-12)

### Features

* Add some methods to VoxelGrid ([63addfd](https://github.com/luoxuhai/pcl.js/commit/63addfd95c0aba26a90576c5954368b0361b4c29))
* Add API documentation([45a0170](https://github.com/luoxuhai/pcl.js/commit/45a0170bcea593e605660a54410884d0ef202115))


## [1.1.1](https://github.com/luoxuhai/pcl.js/compare/v1.1.0...v1.1.1) (2022-09-11)


### Bug Fixes

* filters and registration missing point type ([a611688](https://github.com/luoxuhai/pcl.js/commit/a611688291434a2aa2397071a55f4e38697cae88))


# [1.1.0](https://github.com/luoxuhai/pcl.js/compare/v1.0.3...v1.1.0) (2022-09-10)


### Bug Fixes

* correct typescript type hints ([95de0d3](https://github.com/luoxuhai/pcl.js/commit/95de0d3f7dc274681d97d7d17ac3953a34369a93))


### Features

* add point types ([769f029](https://github.com/luoxuhai/pcl.js/commit/769f0290d090d600131edd7b9b396038895e75d5))
* add point_types ([c5fc1ca](https://github.com/luoxuhai/pcl.js/commit/c5fc1caba2ce10e0088ec369585d07c1a485bd2c))
* add points types ([deb9e7c](https://github.com/luoxuhai/pcl.js/commit/deb9e7c896e34aa24f6ed278efaf293cc480d21f))
* add print version ([0f1c799](https://github.com/luoxuhai/pcl.js/commit/0f1c799d2c1d55ec8387f62c2b22528b66e95691))
* add readPCDHeader method ([4d5d16b](https://github.com/luoxuhai/pcl.js/commit/4d5d16ba9594043ab36089dc1dd3bf194210447a))
* allow Registration.align method to carry parameters ([738b915](https://github.com/luoxuhai/pcl.js/commit/738b9153dbd86f87ffe359253252f441fc7f8f2c))
* get points of point cloud ([c492ac2](https://github.com/luoxuhai/pcl.js/commit/c492ac23f913be491c93aa2585fab31713591550))
* use `Class` to define `PointType` ([51c11ea](https://github.com/luoxuhai/pcl.js/commit/51c11ea6f71c38028dd662bb1bfdfdfd9fc2e9e8))

# [1.1.0-alpha.1](https://github.com/luoxuhai/pcl.js/compare/v1.1.0-alpha.0...v1.1.0-alpha.1) (2022-09-07)


### Features

* allow Registration.align method to carry parameters ([e6514cd](https://github.com/luoxuhai/pcl.js/commit/e6514cd5ec1badd68a7411511148c7347c34093a))
* use `Class` to define `PointType` ([ec57cc2](https://github.com/luoxuhai/pcl.js/commit/ec57cc23f843a0eb6af8d1c8c2652538c602452a))

# [1.1.0-alpha.0](https://github.com/luoxuhai/pcl.js/compare/v1.0.3...v1.1.0-alpha.0) (2022-09-04)


### Bug Fixes

* correct typescript type hints ([d2e952f](https://github.com/luoxuhai/pcl.js/commit/d2e952fbe0f0d79cec1530752015088499328649))


### Features

* add point types ([7d392e4](https://github.com/luoxuhai/pcl.js/commit/7d392e4eefd50f4d9f7b3d6f10518be2d2c58e7d))
* add point_types ([41a7882](https://github.com/luoxuhai/pcl.js/commit/41a78823f21b3282996104ff59ad98d82cf0323d))
* add points types ([7fbfcad](https://github.com/luoxuhai/pcl.js/commit/7fbfcad42fc988b262e9de632f0170daa137456b))
* add print version ([5af8c10](https://github.com/luoxuhai/pcl.js/commit/5af8c105dc190d632cc386cef0214ae9ceb3819a))
* get points of point cloud ([a62d6fe](https://github.com/luoxuhai/pcl.js/commit/a62d6fecb6df20d2abbb1768c04aaef77cfe3fa3))

# 1.0.3 (2022-08-30)


### Features

* Add getFilterLimits methods to PassThrough ([2351435](https://github.com/luoxuhai/pcl.js/commit/23514350a5f4fc2ec6aa5ba822eb7cd5ec49fe3f))
* Bind IterativeClosestPoint of PCL to javascript ([c90df44](https://github.com/luoxuhai/pcl.js/commit/c90df444a1dfce3844b984e4bf986f8b94a2accd))

# 1.0.2 (2022-08-27)


### Bug Fixes

* fix npm package invalid ([8795e8b](https://github.com/luoxuhai/pcl.js/commit/8795e8b46b1c087fa0de6e52553d5a7cf79c3721))

# 1.0.0 (2022-08-27)


### Features

* add some fs methods to pcl object ([6d404b9](https://github.com/luoxuhai/pcl.js/commit/6d404b968a9a268c715f355715869a520ef3c108))
* add some options to initPCL ([2001c02](https://github.com/luoxuhai/pcl.js/commit/2001c0209cb70647daf89b870496a0a49d6af080))
* add some options to initPCL ([515816e](https://github.com/luoxuhai/pcl.js/commit/515816e6023a56d7e9cf8acab4207c75f34ab681))
* add website ([49159a9](https://github.com/luoxuhai/pcl.js/commit/49159a9c8eb3d407f9f389102e4f714fae020be6))
* bind IterativeClosestPoint of PCL to javascript ([a708734](https://github.com/luoxuhai/pcl.js/commit/a7087345f175d5b3cf4a869b9d7862f538d9873b))
* bind loadPCDFile of PCL to javascript ([b2ce4b1](https://github.com/luoxuhai/pcl.js/commit/b2ce4b120514532758629e614bf02b79991a86a6))
* bind PassThrough of PCL to javascript ([77c092b](https://github.com/luoxuhai/pcl.js/commit/77c092bc8fe8a1667960fb179239acd513abc322))
* bind RadiusOutlierRemoval of PCL to javascript ([e33a2f6](https://github.com/luoxuhai/pcl.js/commit/e33a2f6a527daf048262a1549a3dccdfb9232de5))
* bind savePCDFile of PCL to javascript ([d3be8ad](https://github.com/luoxuhai/pcl.js/commit/d3be8add608c826bf8fb8b8c83265114f7e842f5))
* bind StatisticalOutlierRemoval of PCL to javascript ([e195c88](https://github.com/luoxuhai/pcl.js/commit/e195c884e105ed9f3f52b4374d7a72009988f047))
* bind VoxelGrid of PCL to javascript ([5c8f35e](https://github.com/luoxuhai/pcl.js/commit/5c8f35edce01f2a1a42555bf596000ad6c6a15ec))
* build ([3113828](https://github.com/luoxuhai/pcl.js/commit/3113828513064fb9337fcf088e29948be63b4ce6))
* connecting pcl and javascript ([b714a8f](https://github.com/luoxuhai/pcl.js/commit/b714a8f603b99c0ac2c2baf73d6b9e16476b0e10))
* init ([b352f3b](https://github.com/luoxuhai/pcl.js/commit/b352f3b156060e14f4d118adc68611e13829ce5b))