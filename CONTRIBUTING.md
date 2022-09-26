# Contributing Guide

Thanks for the help! We welcome all contributions to it. 

## Project Layout

- [`src`](/src) All the source code for pcl.js, if you want to edit a api or just see how it works this is where you'll find it.
- [`website`](/website) The source code for https://pcljs.org
- [`tests`](/tests) You'll do most of your testing in here.
- [`core`](/core) Point Cloud Library (PCL) project source code.

## Download and Setup

1. [fork](https://docs.github.com/cn/get-started/quickstart/fork-a-repo#forking-a-repository) this repository to your Github, then clone to your local device (If you want to change C++ code, use `--recursive` for cloning submodules) or use [Gitpod](https://gitpod.io/#https://github.com/luoxuhai/pcl.js), a free online dev environment for GitHub.

```bash
git clone https://github.com/luoxuhai/pcl.js --recursive
````

Or discard most branches and history to clone faster:
```bash
git clone https://github.com/luoxuhai/pcl.js --recursive --depth 1 --single-branch --branch master
````

2. Install [Node LTS](https://nodejs.org/en/download/)
3. Install the [Emscripten SDK](https://emscripten.org/docs/getting_started/downloadshtml#installation-instructions-using-the-emsdk-recommended) (No need to install if you don't change C++ code, such as files under `src/bind` and `core/`)

## Editing

1. Compile PCL(C++), [README](https://github.com/luoxuhai/pcl/tree/wasm/wasm/README.md)
2. Compile WebAssembly if C++ code is changed: `npm run dev:wasm` or `npm run build:wasm`
3. Package JavaScript `npm run dev:js` or `npm run build:js`

## Testing Your Changes

1. Create the test code in the `/tests` directory (if the file does not already exist, create it with the `*.test.ts` extension)
2. If you change the C++ code, you need to run `npm run dev:wasm` or `npm run build:wasm`
2. Run `npm run dev:js` or `npm run build:js` to build the test run code
3. Run `npm run test`

## Updating Documentation

Our documentation is made with [Docusaurus](https://docusaurus.io/). They are located in the `website/docs/` directory.

1. Open the `website/` directory, `cd website`
1. Install dependencies `npm install` or `yarn`
2. Start the project `npm run start`, for Chinese: `npm run start:zh`
3. Build the project `npm run build` to check locally for errors
   
## Writing a Commit Message

Before you create a Pull Request, please check whether your commits comply with
the commit conventions used in this repository.

When you create a commit we kindly ask you to follow the convention
`category(scope or module): message` in your commit message while using one of
the following categories:

- `feat`: all changes that introduce completely new code or new
  features
- `fix`: changes that fix a bug (ideally you will additionally reference an
  issue if present)
- `refactor`: any code related change that is not a fix nor a feature
- `perf`: Code changes to improve performance
- `docs`: changing existing or creating new documentation (i.e. README, docs for
  usage of a lib or cli usage)
- `build`: all changes regarding the build of the software, changes to
  dependencies or the addition of new dependencies
- `test`: all changes regarding tests (adding new tests or changing existing
  ones)
- `style`: Changes that do not affect the meaning of the code (whitespace, formatting, missing semicolons, etc.)
- `chore`: all changes to the repository that do not fit into any of the above
  categories

If you are interested in the detailed specification you can visit
[Conventional Commits](https://www.conventionalcommits.org) or check out the
[Angular Commit Message Guidelines](https://github.com/angular/angular/blob/22b96b9/CONTRIBUTING.md#-commit-message-guidelines).

## Command introduction

- `npm run build:pcl` compile PCL to WebAssembly static library
- `npm run dev:js` to package a debug version of JavaScript code
- `npm run dev:wasm` to compile PCL's WebAssembly + `src/bind` to WebAssembly debug build
- `npm run dev` simultaneously packages JavaScript and compiles PCL for development use
- `npm run build:js` to package the production version of JavaScript code
- `npm run build:wasm` to compile PCL's WebAssembly + `src/bind` to WebAssembly production build
- `npm run build` bundles JavaScript and compiles PCL at the same time, ready for production use
- `npm run lint` to check code style
- `npm run test` to run tests. (Be sure to run `npm run dev:js` or `npm run build:js` before running tests)