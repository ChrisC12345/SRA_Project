function K_outer_hybrid = k_matrix_solver_outer_hybrid()
A_outer = [
    0,0,0,1,0,0
    0,0,0,0,1,0
    0,0,0,0,0,1
    0,0,0,0,0,0
    0,0,0,0,0,0
    0,0,0,0,0,0

    ];
B_outer = [
    0,0,0
    0,0,0
    0,0,0
    1,0,0
    0,1,0
    0,0,1
    ];
Q_outer = diag([1,1,1,3,3,3]);
R_outer = diag([0.1,0.1,0.01]);

K_outer_hybrid = lqr(A_outer,B_outer,Q_outer,R_outer);
end