/** @type {import('ts-jest/dist/types').InitialOptionsTsJest} */
module.exports = {
  preset: 'ts-jest',
  verbose: true,
  testEnvironment: 'node',
  testRegex: '(/tests/.*(\\.|/)(test|spec))\\.[jt]s$',
};
