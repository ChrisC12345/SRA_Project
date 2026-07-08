function [thrust] = solve_thrust(z_ddot, z_dot, euler_angles)
    m = ;
    g = ;
    C_d = ;
    phi = euler_angles(1);
    theta = euler_angles(2);
    thrust = (m*z_ddot + z_drag*z_dot + mg)/(cos(phi)*cos(theta));
end


function [phi_ref, theta_ref] = solve_roll_pitch(pos_ddot, pos_dot, euler_angles, F)
    arguments
        pos_ddot (3,1)
        pos_dot (3,1)
        euler_angles (3,1)
        F (1,1)
    end
    m = ;
    x_drag = ;
    y_drag = ;
    x_ddot, y_ddot, z_ddot = pos_ddot;
    x_dot, y_dot, z_dot = pos_dot;
    phi, theta, psi = euler_angles;
    a = (m*x_ddot + x_drag*x_dot)/F;
    b = (m*y_ddot + y_drag*y_dot)/F;
    phi_ref = asin(a*sin(psi) - b*cos(psi));
    theta_ref = asin(a*cos(psi) + b*sin(psi));
end