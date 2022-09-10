# 贡献指南

感谢您的帮助！我们欢迎所有贡献者。

## 下载和安装

1. [fork](https://docs.github.com/cn/get-started/quickstart/fork-a-repo#forking-a-repository) 这个仓库到你的 Github，然后克隆到你的本地设备 （使用 `--recursive` 用于克隆子模块）

```bash
git clone https://github.com/luoxuhai/pcl.js --recursive
```

或者丢弃大部分分支和历史来更快地克隆：
```bash

git clone https://github.com/luoxuhai/pcl.js --recursive --depth 1 --single-branch --branch master
```

2. 安装 [Node LTS](https://nodejs.org/en/download/)
3. 安装 [Emscripten SDK](https://emscripten.org/docs/getting_started/downloadshtml#installation-instructions-using-the-emsdk-recommended)，如果你需要更改 C++ 模块 (`/core/pcl`)。macOS: `brew install emscripten`

### 安装依赖

```bash
cd pcl.js
npm install
```

## 编写代码

1. 编译 PCL(C++)，[README](https://github.com/luoxuhai/pcl/tree/wasm/wasm/README.md)
2. 编译 WebAssembly `npm run dev:wasm`
3. 打包 JavaScript `npm run dev:js` 实时监听 JavaScript 更改 

## 测试你的更改

1. 在 `/tests` 目录下创建测试代码（如果该文件尚不存在，请使用 `*.test.ts` 扩展名创建它）
2. 运行 `npm run dev` 或 `npm run build` 用于构建测试运行的代码
3. 运行 `npm run test`

## 更新文档

我们的文档是用 [Docusaurus](https://docusaurus.io/) 制作的。它们位于 `website/docs/` 目录中。

1. 打开 `website/` 目录(`cd website`)
1. 安装依赖 `npm install` 或者 `yarn`
2. 启动项目 `npm run start`，对于中文：`npm run start:zh`
3. 构建项目 `npm run build`，用于在本地检查是否存在错误

## 编写 Commit Message

在创建拉取请求之前，请检查您的提交是否符合此存储库中使用的提交约定。

当您创建提交时，请遵守约定：`category(scope or module): message`，其中 `category` 是以下之一：

-  `feat`：一项新的功能特性
-  `fix`：修复 bug
-  `refactor`：重构（既不修复bug也不增加新功能的代码更改）
-  `perf`：提高性能的代码更改
-  `docs`：更改现有文档或创建新文档
-  `build`：影响构建系统或外部依赖项的更改（示例范围: rollup、npm）
-  `test`： 添加缺失的测试或更正现有的测试
-  `style`：不会影响代码含义的更改（空格，格式，缺少分号等）
-  `chore`：不符合上述任何一项的对存储库的更改

如果您对详细规范感兴趣，可以访问 [Conventional Commits](https://www.conventionalcommits.or) 或查看 [Angular Commit Message Guidelines](https://github.com/angular/angular/blob/22b96b9/CONTRIBUTING.md#-commit-message-guidelines)。