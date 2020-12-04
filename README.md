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
import * as mcht from 'homogeneous-transformations';
import * as hlao from 'matrix-computations';

var T0 = hlao.matrix_multiplication(
                hlao.matrix_multiplication(
                    hlao.matrix_multiplication(
                        mcht.transl(1,2,3),
                        mcht.trotx(1)
                    ),
                    mcht.troty(1)
                ),
                mcht.trotz(1)
            );
var T1 = hlao.matrix_multiplication(
                hlao.matrix_multiplication(
                        hlao.matrix_multiplication(
                            hlao.matrix_multiplication(
                                T0,
                                mcht.transl(0.01,0.02,0.03)
                            ),
                            mcht.trotx(0.001)
                        ),
                        mcht.troty(0.002)
                    ),
                mcht.trotz(0.003)
            );
var d = mcht.tr2delta(T0,T1);
console.log(d);
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
  [ 0.0191],
  [-0.0113],
  [ 0.0301],
  [ 0.0019],
  [-0.0011],
  [ 0.0030]
]
```