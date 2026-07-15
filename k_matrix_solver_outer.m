function K_outer = k_matrix_solver_outer()
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

    Q_outer = diag([4,4,8,1,1,2]);
    R_outer = diag([0.1,0.1,0.1]);
    
    K_outer = lqr(A_outer,B_outer,Q_outer,R_outer);

    %disp(eig(A_outer - B_outer * K_outer));
end