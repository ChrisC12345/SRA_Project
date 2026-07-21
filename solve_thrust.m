function [thrust] = solve_thrust(z_ddot, z_dot, euler_angles)
    p = params();
    m = p.m; 
    g = p.g;
    z_drag = 0;
    phi = euler_angles(1);
    theta = euler_angles(2);
    d = cos(phi)*cos(theta);
    if abs(d) < 0.01
        d = 0.01*sign(d);
    end
    thrust = (m*z_ddot + z_drag*z_dot + m*g)/(d);
end
