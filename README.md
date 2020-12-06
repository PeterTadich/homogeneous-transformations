# homogeneous-transformations
Homogeneous transformations - position/orientation

## Dependencies

There is 3 dependency 'matrix-computations', 'elementary-rotations' and 'quaternions'.

```bash
https://github.com/PeterTadich/matrix-computations
https://github.com/PeterTadich/elementary-rotations
https://github.com/PeterTadich/quaternions
```

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

## Examples (extended)

Cartesian Motion: linear translation and spherical rotation (slerp) interpolation - trinterp().

```bash
npm install https://github.com/PeterTadich/trajectories https://github.com/PeterTadich/homogeneous-transformations
```

```js
import * as hlao from 'matrix-computations';
import * as mcht from 'homogeneous-transformations';
import * as mcer from 'elementary-rotations';
import * as mcqt from 'quaternions';
import * as ttvm from 'trajectories';

//create homogeneous transformations
var T0 = mcht.trotx(Math.PI);
var T1 = hlao.matrix_multiplication(
            mcht.troty(Math.PI/2.0),
            mcht.trotz(-1.0*Math.PI/2.0)
        );

//setup time-steps
var qi = 0.0; var qf = 1.0; var tf = 1.0; var nsteps = 100; 
var s = ttvm.lspb(qi,qf,tf,nsteps);

//run interpolation 'trinterp()'
var T = [];
for(var i=0;i<s.length;i=i+1){
    T.push(mcht.trinterp(T0,T1,s[i][1]));
    if((s[i][1] > 0.495)&&(s[i][1] < 0.505)) console.log(T[i]);
}
```

Returns:

```js
//at time-step 0.5
[
  [ 0.0,  1.0,  0.0,  0.0],
  [ 0.0,  0.0, -1.0,  0.0],
  [-1.0,  0.0,  0.0,  0.0],
  [ 0.0,  0.0,  0.0,  1.0],
]
```