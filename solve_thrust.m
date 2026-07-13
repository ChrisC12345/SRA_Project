function [thrust] = solve_thrust(z_ddot, z_dot, euler_angles)
    m = 1; %temp
    g = 9.81;
    z_drag = 0;
    phi = euler_angles(1);
    theta = euler_angles(2);
    d = cos(phi)*cos(theta);
    if abs(d) < 0.1
        d = 0.1*sign(d);
    end
    thrust = (m*z_ddot + z_drag*z_dot + m*g)/(d);
    thrust = clip(thrust, 1, 20);
end
