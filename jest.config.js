/** @type {import('ts-jest/dist/types').InitialOptionsTsJest} */
module.exports = {
  preset: 'ts-jest',
  verbose: true,
  runner: '@kayahr/jest-electron-runner',
  testEnvironment: '@kayahr/jest-electron-runner/environment',
  testRegex: '(/tests/.*(\\.|/)(test|spec))\\.[jt]s$',
};
