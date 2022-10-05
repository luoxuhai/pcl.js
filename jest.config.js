/** @type {import('ts-jest/dist/types').InitialOptionsTsJest} */
module.exports = {
  preset: 'ts-jest',
  verbose: true,
  testEnvironment: 'jsdom',
  testRegex: '(/tests/.*(\\.|/)(test|spec))\\.[jt]s$',
  setupFilesAfterEnv: ['<rootDir>/tests/setup-tests.ts'],
  testTimeout: 1000 * 60,
};
