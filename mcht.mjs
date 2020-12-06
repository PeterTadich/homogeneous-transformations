//mcht = matrix computations homogeneous transformations

//ECMAScript module

//npm install https://github.com/PeterTadich/elementary-rotations https://github.com/PeterTadich/quaternions https://github.com/PeterTadich/matrix-computations#main 

import * as hlao from 'matrix-computations';
import * as mcer from 'elementary-rotations';
import * as mcqt from 'quaternions';

function transl(x,y,z){
    var T = [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ];
    
    return T;
}

//rotation about X axis (radians)
function trotx(theta){
    var Rx = mcer.Rx_elementary(theta);
    var T = [
        [Rx[0][0], Rx[0][1], Rx[0][2], 0.0],
        [Rx[1][0], Rx[1][1], Rx[1][2], 0.0],
        [Rx[2][0], Rx[2][1], Rx[2][2], 0.0],
        [     0.0,      0.0,      0.0, 1.0]
    ];
    
    return T;
}

//rotation about Y axis (radians)
function troty(theta){
    var Ry = mcer.Ry_elementary(theta);
    var T = [
        [Ry[0][0], Ry[0][1], Ry[0][2], 0.0],
        [Ry[1][0], Ry[1][1], Ry[1][2], 0.0],
        [Ry[2][0], Ry[2][1], Ry[2][2], 0.0],
        [     0.0,      0.0,      0.0, 1.0]
    ];
    
    return T;
}

//rotation about Z axis (radians)
function trotz(theta){
    var Rz = mcer.Rz_elementary(theta);
    var T = [
        [Rz[0][0], Rz[0][1], Rz[0][2], 0.0],
        [Rz[1][0], Rz[1][1], Rz[1][2], 0.0],
        [Rz[2][0], Rz[2][1], Rz[2][2], 0.0],
        [     0.0,      0.0,      0.0, 1.0]
    ];
    
    return T;
}

//roll, pitch, yaw to homogeneous transformation matrix corresponding to rotation (radians)
//IMPORTANT: returns solution for sequential rotations about Z, Y, X axes (Paul book)
//REF: Robotics, Vision and Control. Page 30
/*
MATLAB
T = rpy2tr(0.1,0.2,0.3,'zyx')
T =
    0.9752   -0.0370    0.2184         0
    0.0978    0.9564   -0.2751         0
   -0.1987    0.2896    0.9363         0
         0         0         0    1.0000

JavaScript
var T = rpy2tr(0.1, 0.2, 0.3);
console.log(T);
T = [
    [   0.975170327201816, -0.036957013524625076, 0.21835066314633442, 0.0],
    [ 0.09784339500725571,    0.9564250858492325, -0.2750958473182437, 0.0],
    [-0.19866933079506122,   0.28962947762551555,  0.9362933635841992, 0.0],
    [                 0.0,                   0.0,                 0.0, 1.0]
]
*/
function rpy2tr(roll, pitch, yaw){
    var Rrpy = mcer.rpy2r(roll, pitch, yaw);
    var T = [
        [Rrpy[0][0], Rrpy[0][1], Rrpy[0][2], 0.0],
        [Rrpy[1][0], Rrpy[1][1], Rrpy[1][2], 0.0],
        [Rrpy[2][0], Rrpy[2][1], Rrpy[2][2], 0.0],
        [       0.0,        0.0,        0.0, 1.0]
    ];
    
    return T;
}

function trinterp(T0,T1,t){
    //rotation
    //   - extract rotation matrix
    var R0 = [
        [T0[0][0],T0[0][1],T0[0][2]],
        [T0[1][0],T0[1][1],T0[1][2]],
        [T0[2][0],T0[2][1],T0[2][2]]
    ];
    var R1 = [
        [T1[0][0],T1[0][1],T1[0][2]],
        [T1[1][0],T1[1][1],T1[1][2]],
        [T1[2][0],T1[2][1],T1[2][2]]
    ];
    //   - convert to quaternions
    var q0 = mcqt.unitQuaternionFromRotationMatrix(R0);
    var q1 = mcqt.unitQuaternionFromRotationMatrix(R1);
    //   - interpolate
    var q = mcqt.quaternionSlerp(q0,q1,t);
    //   - convert to rotation matrix
    var R = mcqt.rotationMatrixFromUnitQuaternion(q);

    //translation
    //   - extract position
    var p0 = [[T0[0][3]],[T0[1][3]],[T0[2][3]]];
    var p1 = [[T1[0][3]],[T1[1][3]],[T1[2][3]]];
    //   - interpolate
    var p = hlao.matrix_arithmetic(
                hlao.vector_multiplication_scalar(p0,(1-t)),
                hlao.vector_multiplication_scalar(p1,t),
                '+'
            );
    
    //build the homogenous transform
    var T = [
        [R[0][0],R[0][1],R[0][2],p[0][0]],
        [R[1][0],R[1][1],R[1][2],p[1][0]],
        [R[2][0],R[2][1],R[2][2],p[2][0]],
        [    0.0,    0.0,    0.0,    1.0]
    ];
    return(T);
}

//convert:
//   - individual coordinate transformations to
//   - coordinate transformation describing the position and orientation of Frame n with respect to Frame 0
/*
var TOn = CTdIF(Links);
*/
//ref: DenavitHartenberg.js
function CTdIF(Li){ //Coordinate Transform described in Inertial Frame
    var TOn = []; //"O" for Oscar
    for(var i=0;i<Li.length;i=i+1){
        if(i == 0) TOn[i] = Aij(Li[i]);
        else TOn[i] = hlao.matrix_multiplication(TOn[i-1],Aij(Li[i])); //Aij - transforms a vector from frame {j} to frame {i}
    }
    return TOn;
}

//ref: DenavitHartenberg.js
function Aij(Li){ //Ai-1,j
    var debug = 0;
    
    // Li = [ai, alpha_i, di, vi]
    var ai = Li[0];
    var alpha_i = Li[1];
    var di = Li[2];
    var vi = Li[3];
    var EPS = 0.001;
    
    var Aip = [ //Ai',i-1
        [Math.cos(vi),-1.0*Math.sin(vi), 0.0, 0.0],
        [Math.sin(vi),     Math.cos(vi), 0.0, 0.0],
        [      0.0,                 0.0, 1.0,  di],
        [      0.0,                 0.0, 0.0, 1.0]
    ];

    var Ai = [ //Ai,i'
        [      1.0,               0.0,                   0.0,  ai],
        [      0.0, Math.cos(alpha_i),-1.0*Math.sin(alpha_i), 0.0],
        [      0.0, Math.sin(alpha_i),     Math.cos(alpha_i), 0.0],
        [      0.0,               0.0,                   0.0, 1.0]
    ];
    
    if(debug) console.log(Aip);
    if(debug) console.log(Ai);
    
    var Aiqi = hlao.matrix_multiplication(Aip,Ai);
    
    //check
    var Ai_neg1 = [
        [Math.cos(vi), -1.0*Math.sin(vi)*Math.cos(alpha_i),      Math.sin(vi)*Math.sin(alpha_i), ai*Math.cos(vi)],
        [Math.sin(vi),      Math.cos(vi)*Math.cos(alpha_i), -1.0*Math.cos(vi)*Math.sin(alpha_i), ai*Math.sin(vi)],
        [         0.0,                   Math.sin(alpha_i),                   Math.cos(alpha_i),              di],
        [         0.0,                                 0.0,                                 0.0,             1.0]
    ]
    
    if(debug) console.log(Aiqi);
    if(debug) console.log(Ai_neg1);
    
    for(var i=0;i<4;i=i+1){
        for(var j=0;j<4;j=j+1){
            hlao.assert((Math.abs(Math.abs(Aiqi[i][j]) - Math.abs(Ai_neg1[i][j])) < EPS),'Assertion failed: coordinate transform error, Aij().'); //hlao.mjs ('matrix-computations')
        }
    }
    
    return Aiqi;
}

//homogeneous transformation matrix - inverse
//REF: Robotics Modelling, Planning and Control, Page 57
//ref: DenavitHartenberg.js
function HTInverse(T){
    /*
    var R = [
        [T[0][0],T[0][1],T[0][2]],
        [T[1][0],T[1][1],T[1][2]],
        [T[2][0],T[2][1],T[2][2]]
    ];
    */
    
    var RT = [
        [T[0][0],T[1][0],T[2][0]],
        [T[0][1],T[1][1],T[2][1]],
        [T[0][2],T[1][2],T[2][2]]
    ];
    
    var o = [[T[0][3]],[T[1][3]],[T[2][3]]];
    
    o = hlao.matrix_multiplication(hlao.matrix_multiplication_scalar(RT,-1.0),o);
    
    var Tin = [
        [RT[0][0],RT[0][1],RT[0][2],o[0][0]],
        [RT[1][0],RT[1][1],RT[1][2],o[1][0]],
        [RT[2][0],RT[2][1],RT[2][2],o[2][0]],
        [     0.0,     0.0,     0.0,    1.0]
    ];
    
    return Tin;
}

//R - rotation matrix. o - origin of R
function homogeneousTransformationMatrix(R,o){
    /*
           -         -
           |R0,1 o0,1|
    A0,1 = |         |     Equ. 2.42
           |  0T    1|
           -         -
    
    A0,1 - transformation of a vector from Frame 1 to Frame 0.
         - the homogeneous transformation matrix contains:
           - 'R0,1' rotation matrix of Frame 1 with respect to Frame 0
           - 'o0,1' translation vector from the origin of Frame 0 to origin of Frame 1
    
    REF: Robotics Modelling, Planning and Control, Page 57
    */
    return([
        [R[0][0],R[0][1],R[0][2],o[0][0]],
        [R[1][0],R[1][1],R[1][2],o[1][0]],
        [R[2][0],R[2][1],R[2][2],o[2][0]],
        [    0.0,    0.0,    0.0,    1.0]
    ]);
}

//REF: Robotics, Vision and Control. Page 53.
/*
//MATLAB:
T0 = transl(1,2,3)*trotx(1)*troty(1)*trotz(1)

T0 =

    0.2919   -0.4546    0.8415    1.0000
    0.8372   -0.3039   -0.4546    2.0000
    0.4624    0.8372    0.2919    3.0000
         0         0         0    1.0000

>> T1 = T0*transl(0.01,0.02,0.03)*trotx(0.001)*troty(0.002)*trotz(0.003)

T1 =

    0.2889   -0.4547    0.8425    1.0191
    0.8372   -0.3069   -0.4527    1.9887
    0.4644    0.8361    0.2920    3.0301
         0         0         0    1.0000

 d = tr2delta(T0,T1)'

d =

    0.0191   -0.0113    0.0301    0.0019   -0.0011    0.0030

//JavaScript:
T0 = [
    [0.2919,   -0.4546,    0.8415,    1.0000],
    [0.8372,   -0.3039,   -0.4546,    2.0000],
    [0.4624,    0.8372,    0.2919,    3.0000],
    [     0,         0,         0,    1.0000]
];
T1 = [
    [0.2889,   -0.4547,    0.8425,    1.0191],
    [0.8372,   -0.3069,   -0.4527,    1.9887],
    [0.4644,    0.8361,    0.2920,    3.0301],
    [     0,         0,         0,    1.0000]
];
d = tr2delta(T0,T1);
console.log(d);
*/
//with ref. to world frame and for Incremental Motion see Robotics Vision and Control, page 52. (page 53, equ. 3.10)
//ref: DenavitHartenberg.js
function tr2delta(T0,T1){
    var t0 = [[T0[0][3]],[T0[1][3]],[T0[2][3]]];
    var t1 = [[T1[0][3]],[T1[1][3]],[T1[2][3]]];
    var R0 = [
        [T0[0][0],T0[0][1],T0[0][2]],
        [T0[1][0],T0[1][1],T0[1][2]],
        [T0[2][0],T0[2][1],T0[2][2]]
    ];
    var R1 = [
        [T1[0][0],T1[0][1],T1[0][2]],
        [T1[1][0],T1[1][1],T1[1][2]],
        [T1[2][0],T1[2][1],T1[2][2]]
    ];
    
    var t = hlao.matrix_arithmetic(t1,t0,'-');
    var v = infinitesimalRotation(R0,R1);

    //d=(dx, dy, dz, dRx, dRy, dRz)
    var d = [[t[0][0]],[t[1][0]],[t[2][0]],[v[0][0]],[v[1][0]],[v[2][0]]];
    return d;
}

//page 53 ref: Robotics, Vision and Control
//MATLAB
/*
T0 = transl(1,2,3) * trotx(1) * troty(1) * trotz(1);
T1 = T0 * transl(0.01,0.02,0.03) * trotx(0.001) * troty(0.002) * trotz(0.003);
d = tr2delta(T0, T1);
delta2tr(d) * T0
    0.2889   -0.4547    0.8425    1.0096
    0.8372   -0.3069   -0.4527    1.9859
    0.4644    0.8361    0.2920    3.0351
         0         0         0    1.0000

//JavaScript
var T0 = matrix_multiplication(matrix_multiplication(matrix_multiplication(transl(1,2,3),trotx(1)),troty(1)),trotz(1));
var T1 = matrix_multiplication(matrix_multiplication(matrix_multiplication(matrix_multiplication(T0,transl(0.01,0.02,0.03)),trotx(0.001)),troty(0.002)),trotz(0.003));
console.log(T1);
var d = tr2delta(T0,T1); //see file 'DenavitHartenberg.js'
console.log(d);
var T = delta2tr(d);
console.log(T);
console.log(matrix_multiplication(T,T0));
*/
//ref: DenavitHartenberg.js
function delta2tr(d){
    //var dtws = S([[d[3][0]],[d[4][0]],[d[5][0]]]); //delta theta world skew (incremental rotation)
    var dtws = hlao.skew([[d[3][0]],[d[4][0]],[d[5][0]]]); //delta theta world skew (incremental rotation)
    var E = hlao.identity_matrix(4); //4x4
    var T = hlao.matrix_arithmetic(
                [
                    [dtws[0][0],dtws[0][1],dtws[0][2],d[0][0]], //equ.(3.12)
                    [dtws[1][0],dtws[1][1],dtws[1][2],d[1][0]],
                    [dtws[2][0],dtws[2][1],dtws[2][2],d[2][0]],
                    [      0.0,        0.0,       0.0,    0.0]
                ],
                E,
                '+'
            );
    return T;
}

//page 52 ref: Robotics, Vision and Control
//return 3-vector (with units of angle) that represents an infinitesimal rotation about the world x-, y- and z-axes
//ref: DenavitHartenberg.js
function infinitesimalRotation(R0,R1){
    var R0T = hlao.matrix_transpose(R0); //R0 transpose
    var I33 = hlao.identity_matrix(3);
    
    return hlao.vex(
        hlao.matrix_arithmetic(
            hlao.matrix_multiplication(R1,R0T),
            I33,
            '-'
            )
        );
}

//Deep copy of the Denavit-Hartenberg parameters.
//ref: DenavitHartenberg.js
function DHDeepCopy(DH){
    //[ai, alpha_i, di, vi]
    var DHcopy = []
    for(var i=0;i<DH.length;i=i+1){
        DHcopy[i] = [];
        for(var j=0;j<DH[i].length;j=j+1){
            DHcopy[i][j] = DH[i][j];
        }
    }
    return DHcopy;
}

//ref: DenavitHartenberg.js
function directKinematicsDH(Li){
    var debug = 1;

    // compute the direct kinematic function Tb,e(q) using Denavit-Hartenberg parameters
    // REF: Robotics Modelling, Planning and Control, Page 64
    var EPS = 0.001;

    var Tb0 = hlao.identity_matrix(4); //transforms a vector from frame {0} to frame {b} (base frame)
    var Tne = [ //transforms a vector from frame {e} (end-effector) to frame {n} (link n)
        [ 0.0, 0.0, 1.0, 0.0],
        [ 0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [ 0.0, 0.0, 0.0, 1.0]
    ];
    
    //for(var i=0;i<Li.length;i=i+1){
    //    if(i == 0) var T0n = matrix_multiplication(Tb0,Aij(Li[i]));
    //    if((0 < i)&&(i < (Li.length - 1))) T0n = matrix_multiplication(T0n,Aij(Li[i]));
    //    if(i == (Li.length - 1)) T0n = matrix_multiplication(T0n,Tne);
    //}
    
    for(var i=0;i<Li.length;i=i+1){
        if(i == 0) var T0n = Aij(Li[i]);
        else T0n = hlao.matrix_multiplication(T0n,Aij(Li[i])); //Aij - transforms a vector from frame {j} to frame {i}
    }                                                          //T0n - transforms a vector from frame {n} (link n) to frame {0} (link 0)
    
    /*
    //check: three link planar arm only
    var T0nc = threeLinkPlanarArm(Li);
    
    if(debug) console.log(T0n);
    if(debug) console.log(T0nc);
    
    for(var i=0;i<4;i=i+1){
        for(var j=0;j<4;j=j+1){
            hlao.assert((Math.abs(Math.abs(T0n[i][j]) - Math.abs(T0nc[i][j])) < EPS),'Assertion failed: coordinate ttransform error, directKinematicsDH().' + i + ':' + + j);
        }
    }
    */
    
    var Tbe = hlao.matrix_multiplication(hlao.matrix_multiplication(Tb0,T0n),Tne);
    
    return T0n;
}

//see also: I:\code\spatial_v2\js\elementaryRotations\elementaryRotations.js
function print_homogenous_transform(T,x){
    //print the homogenous matrix
    //   - 'T' the homogenous matrix
    //   - 'x' the number of digits after the decimal point to print
    var Tstr = "";
    //var dim = size(T);
    //var m = dim[0]; //number of rows
    //var n = dim[1]; //number of columns
    var m; var n;
    [m,n] = [T.length, T[0].length];
    Tstr = Tstr + '[\n';
    for(var i=0;i<m;i=i+1){ //row
        Tstr = Tstr + '    [';
        for(var j=0;j<n;j=j+1){ //col
            if(j < (n-1)) Tstr = Tstr + T[i][j].toFixed(x) + ', ';
            else Tstr = Tstr + T[i][j].toFixed(x);
        }
        if(i < (m-1)) Tstr = Tstr + '],\n';
        else Tstr = Tstr + ']\n';
    }
    Tstr = Tstr + ']\n';
    console.log(Tstr);
}

function coordinateTransform(){
    // p0~ = A0,1 * p1~
    // where p~ = [p 1]T
    
    // REF: Robotics Modelling, Planning and Control, Page 57
}

export {
    transl,
    trotx,
    troty,
    trotz,
    rpy2tr,
    trinterp,
    CTdIF,
    Aij,
    HTInverse,
    homogeneousTransformationMatrix,
    tr2delta,
    delta2tr,
    infinitesimalRotation,
    DHDeepCopy,
    directKinematicsDH,
    print_homogenous_transform
};