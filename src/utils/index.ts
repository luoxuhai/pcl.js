export const ENVIRONMENT_IS_NODE =
  typeof process === 'object' &&
  typeof process.versions === 'object' &&
  typeof process.versions.node === 'string';

export function getRandomArbitrary(min: number, max: number) {
  return Math.ceil(Math.random() * (max - min) + min);
}
