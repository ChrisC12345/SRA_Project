function parameters = params()

    k_f = 3.0e-5;
    k_m = 7.5e-7;
    l = 0.225;
    wind_speed = 20;

    params.m = 1.0;
    params.g = 9.81;

    params.wind_speed = wind_speed;
    params.V_mean = [wind_speed;0;0];
    params.air_density = 1.225;

    params.arm_length = l;
    params.inertia = [0.01,0,0;0,0.01,0;0,0,0.02];

    params.thrust_coef = k_f;
    params.torque_coef = k_m;
    params.Cd_fuse = 1.0;
    params.Ax = 0.005;
    params.Ay = 0.005;
    params.Az = 0.03;
    params.kh = 1.0e-4;

    A = [ k_f,k_f,k_f,k_f;
        0,-k_f*l,0,k_f*l;
        -k_f*l,0,k_f*l,0;
        -k_m,k_m,-k_m,k_m];

    params.mixer_inv = inv(A);   % precompute the inverse

    parameters = params;

end