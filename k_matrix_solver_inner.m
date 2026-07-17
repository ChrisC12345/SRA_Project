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
    
    Q_inner = diag([80 80 12, 1 1 0.4]);   % angles ~10x, rates ~2x
    R_inner = diag([16 16 100]);             % unchanged
    
    K_inner = lqr(A_inner, B_inner, Q_inner, R_inner);

    damp(eig(A_inner - B_inner * K_inner))
end