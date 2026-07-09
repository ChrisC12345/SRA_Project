function [phi_ref, theta_ref] = solve_roll_pitch(pos_ddot, pos_dot, euler_angles, F)
    arguments
        pos_ddot (3,1)
        pos_dot (3,1)
        euler_angles (3,1)
        F (1,1)
    end
    m = 1;        % temp
    x_drag = 0;
    y_drag = 0;

    x_ddot = pos_ddot(1);
    y_ddot = pos_ddot(2);
    % z_ddot = pos_ddot(3);   % unused

    x_dot = pos_dot(1);
    y_dot = pos_dot(2);
    % z_dot = pos_dot(3);     % unused

    psi = euler_angles(3);    % only psi is used

    a = (m*x_ddot + x_drag*x_dot)/F;
    b = (m*y_ddot + y_drag*y_dot)/F;

    phi_ref   = asin(clip(a*sin(psi) - b*cos(psi),-1,1));

    d = cos(phi_ref);
    if abs(d) < 0.1
        d = 0.1*sign(d);
    end
    theta_ref = asin(clip((a*cos(psi) + b*sin(psi))/d, -1, 1));
end