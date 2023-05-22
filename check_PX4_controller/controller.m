function [control_action, controller_states_next] = controller(states, controller_states, ref)

delta = 0.005;

delta_PX4_pos = 0.02;

omega_c = 2*pi*5; % cutoff frequency

Pv = 5*diag([0.95, 0.95, 1]);
Pr = 5*diag([1.8, 1.8, 4]);
Dv = delta_PX4_pos/delta*diag([0.2, 0.2, 0]);
Iv = delta/delta_PX4_pos*diag([0.4, 0.4, 2]);

p = states(1:3); % current position
v = states(4:6); % current velocity

xi_a_pos = controller_states(1:3);
xi_v_pos = controller_states(4:6);

v_sp = ref.v + Pr*(ref.p - p);

a_sp = ref.a + Pv*(v_sp - v) - Dv*(xi_a_pos + omega_c*v) + Iv*xi_v_pos;

bz = [-a_sp(1); -a_sp(2); 9.81];
bz = bz./norm(bz); % normalize

% ht = 0.532737; % hovering thrust (normalized)
ht = 0.5; % hovering thrust (normalized)

C_T = (a_sp(3)*ht/9.81 - ht)/bz(3);

t_sp = C_T*bz;

Thrust = -norm(t_sp);

b3 = -t_sp./norm(t_sp);
% b2 = cross(b3, [cos(ref.yaw); sin(ref.yaw); 0]);
b1 = cross([-sin(ref.yaw); cos(ref.yaw); 0], b3);
b1 = b1./norm(b1);
Rd = [b1, cross(b3, b1), b3];
% Rd = [cross(b2, b3), b2, b3];

qd = rotm2quat(Rd);



Kq = diag([6.5, 6.5, 2.8]);

q = states(7:10);
q_err = [qd(1)*q(1) + qd(2:4)'*q(2:4);
         q(1)*qd(2:4) - qd(1)*q(2:4) - skew(qd(2:4))*q(2:4)];
om_sp = 2* Kq * sign(q_err(1)) * q_err(2:4);



xi_a_att = controller_states(7:9);
xi_om_att = controller_states(10:12);

om = states(11:13); % current angular velocity

omega_c_att = 2*pi*30; % cutoff frequency

delta_PX4_att = 0.001;

K = 1;
P = diag([0.15, 0.15, 0.2]);
D = delta_PX4_att/delta*diag([0.003, 0.003, 0]);
I = delta/delta_PX4_att*diag([0.2, 0.2, 0.1]);

Torque = K*P*(om_sp - om) - K*D*(xi_a_att + omega_c_att*om) + K*I*xi_om_att;


control_action = [Thrust; Torque];

% integrate next controller states


pos_control_states0 = controller_states(1:6);

[~, xint] = ode45(@(t, controller_states) pos_control_ode(t, states, controller_states, v_sp),[0 delta], pos_control_states0);

pos_control_states = xint(end,:)';
xi_a_pos = pos_control_states(1:3);
xi_v_pos = pos_control_states(4:6);

att_control_states0 = controller_states(7:12);

[~, xint] = ode45(@(t, controller_states) att_control_ode(t, states, controller_states, om_sp),[0 delta], att_control_states0);
att_control_states = xint(end,:)';
xi_a_att = att_control_states(1:3);
xi_om_att = att_control_states(4:6);

controller_states_next = [xi_a_pos; xi_v_pos; xi_a_att; xi_om_att];


end



function pos_control_states = pos_control_ode(~, states, controller_states, v_sp)

    xi_a = controller_states(1:3); 
    v = states(4:6); % current velocity

    omega_c = 2*pi*5; % cutoff frequency
        
    pos_control_states = [-omega_c*xi_a - omega_c^2*v; v_sp - v];
end


function att_control_states = att_control_ode(~, states, controller_states, om_sp)

    xi_a = controller_states(1:3);
    om = states(11:13); % current angular velocity
    
    omega_c = 2*pi*30; % cutoff frequency
        
    att_control_states = [-omega_c*xi_a - omega_c^2*om; om_sp - om];
end