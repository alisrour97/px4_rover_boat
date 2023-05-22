function f_system = dynamics(~,states, u, params)

p = states(1:3);
v = states(4:6);
q = states(7:10); %w,x,y,z
w = states(11:13);

Thrust = u(1);
Torque = u(2:4);

R_bw = quat2rotm(q);

% True model parameters
mass = params(1);
I_diag = params(2:4);
I_mat = diag(I_diag);

% System dynamics
dot_p = v;
dot_v = [0; 0; -9.81] + (1.0/mass) * R_bw * [0; 0; Thrust];
dot_q = [-0.5*q(2:4)'*w; 0.5*( q(1)*eye(3) - skew(q(2:4)) )*w];
dot_w = I_mat \ ( - skew(w) * I_mat*w + Torque );

f_system = [dot_p; dot_v; dot_q; dot_w];

end
