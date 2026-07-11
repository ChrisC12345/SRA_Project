I_xx = 0.005;
I_yy = 0.005;
I_zz = 0.009;

A_inner = [
0,0,0,1,0,0
0,0,0,0,1,0
0,0,0,0,0,1
0,0,0,0,0,0
0,0,0,0,0,0
0,0,0,0,0,0
];

B_inner = [
0,0,0
0,0,0
0,0,0
1/I_xx,0,0
0,1/I_yy,0
0,0,1/I_zz
];

Q_inner = diag([100,100,100,10,10,10]);
R_inner = diag([1,1,1]);

K_inner = lqr(A_inner, B_inner, Q_inner, R_inner);