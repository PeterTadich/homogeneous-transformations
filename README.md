# homogeneous-transformations
homogeneous transformations - position/orientation

## Installation

### Node.js

```bash
npm install https://github.com/PeterTadich/homogeneous-transformations
```

### Google Chrome Web browser

No installation required for the Google Chrome Web browser.

## How to use

### Node.js

```js
import * as mcht from 'homogeneous-transformations';
```

### Google Chrome Web browser

```js
import * as mcht from './mcht.mjs';
```

## Examples

### Node.js (server side)

Copy the following code to index.mjs

```js
import * as mcht from 'pseudo-inverse';

var a = [
    [1,2],
    [3,4],
    [5,6],
    [7,8]
];
var b = mcht.mcht(a);
console.log(b);
```

Then run:

```bash
npm init -y
npm install https://github.com/PeterTadich/homogeneous-transformations
node index.mjs
```

If the above does not work, modify the package.json file as follows:
Helpful ref: [https://stackoverflow.com/questions/45854169/how-can-i-use-an-es6-import-in-node-js](https://stackoverflow.com/questions/45854169/how-can-i-use-an-es6-import-in-node-js)

```js
"scripts": {
    "test": "echo \"Error: no test specified\" && exit 1",
    "start": "node --experimental-modules index.mjs"
  },
"type": "module",
```

Now try:

```bash
npm start
```

Returns:

```js
[
  [ -1.0000,  0.8500 ],
  [ -0.5000,  0.4500 ],
  [  0.0000,  0.0500 ],
  [  0.5000, -0.3500 ]
]
```