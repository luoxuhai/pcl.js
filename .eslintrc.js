module.exports = {
  env: {
    browser: true,
    node: true,
    es6: true,
    jest: true,
  },
  root: true,
  parser: '@typescript-eslint/parser',
  parserOptions: {
    sourceType: 'module',
    ecmaVersion: 2015,
  },
  extends: [
    'standard',
    'plugin:@typescript-eslint/recommended',
    'plugin:prettier/recommended',
  ],
  rules: {
    'prettier/prettier': 'error',
    '@typescript-eslint/triple-slash-reference': 'off',
    '@typescript-eslint/no-explicit-any': 'off',
    'no-useless-constructor': 'off',
  },
  ignorePatterns: [
    '/node_modules/',
    '/dist/',
    '/core/',
    'website',
    '/src/bind/',
  ],
};
