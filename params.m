function parameters = params()

    k_f = 3.0e-5;
    k_m = 7.5e-7;
    l = 0.225;
    wind_speed = 20;
    m = 1.0;
    g = 9.81;

    p.m = m;
    p.g = g;

    p.N = 50;

    p.wind_speed = wind_speed;
    p.V_mean = [wind_speed;0;0];
    p.air_density = 1.225;

    p.arm_length = l;
    p.inertia = [0.01,0,0;0,0.01,0;0,0,0.02];

    p.tau_motor = 0.05;
    p.thrust_coef = k_f;
    p.torque_coef = k_m;
    p.Cd_fuse = 1.0;
    p.Ax = 0.005;
    p.Ay = 0.005;
    p.Az = 0.03;
    p.kh = 1.0e-4;

    A = [ k_f,k_f,k_f,k_f;
        0,-k_f*l,0,k_f*l;
        -k_f*l,0,k_f*l,0;
        -k_m,k_m,-k_m,k_m];

    p.w_hover   = sqrt((m*g/4)/k_f)*ones(4,1); % hover motor speed [rad/s]

    p.mixer_inv = inv(A);   % precompute the inverse

    parameters = p;

end