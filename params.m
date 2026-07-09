function parameters = params()

    k_f = 3.0e-5;
    k_m = 7.5e-7;
    l = 0.225;

    params.m = 1.0;
    params.g = 9.81;
    params.wind_speed = 1.0;
    params.air_density = 1.225;
    params.arm_length = l;
    params.thrust_coef = k_f;
    params.torque_coef = k_m;

    A = [ k_f,k_f,k_f,k_f;
        0,-k_f*l,0,k_f*l;
        -k_f*l,0,k_f*l,0;
        -k_m,k_m,-k_m,k_m];

    params.mixer_inv = inv(A);   % precompute the inverse

    parameters = params;

end