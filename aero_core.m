function [F_tot, M_tot] = aero_core(W, V_rel, w_rel, p)

CdA_matrix = diag([p.Cd_fuse*p.Ax, p.Cd_fuse*p.Ay, p.Cd_fuse*p.Az]);
L  = p.arm_length;
h  = p.rotor_height;

r_matrix = [ L,  0, -L,  0;
    0, -L,  0,  L;
    -h, -h, -h, -h];
dir = [-1, 1, -1, 1];

F_rotors = [0;0;0];
M_rotors = [0;0;0];

for i = 1:4
    V_hub = V_rel + cross(w_rel, r_matrix(:,i));

    H_force  = -p.kh * W(i) * [V_hub(1); V_hub(2); 0];
    Thrust_z =  p.thrust_coef * W(i)^2 - p.kz * W(i) * V_hub(3);

    F_i = [H_force(1); H_force(2); Thrust_z];
    M_i = cross(r_matrix(:,i), F_i) + [0; 0; dir(i)*p.torque_coef*W(i)^2];

    F_rotors = F_rotors + F_i;
    M_rotors = M_rotors + M_i;
end

V_mag = norm(V_rel);
if V_mag > 0
    F_fuse = -0.5 * p.air_density * V_mag * (CdA_matrix * V_rel);
else
    F_fuse = [0;0;0];
end

F_tot = F_rotors + F_fuse;
M_tot = M_rotors;
end