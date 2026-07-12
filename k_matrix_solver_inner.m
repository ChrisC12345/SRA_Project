function K_inner = k_matrix_solver_inner() 
    p = params();
    I_xx = p.inertia(1,1);
    I_yy = p.inertia(2,2);
    I_zz = p.inertia(3,3);
    
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
    
    Q_inner = diag([50,50,50,20,20,20]);
    R_inner = diag([1,1,1]);
    
    K_inner = lqr(A_inner, B_inner, Q_inner, R_inner);
end