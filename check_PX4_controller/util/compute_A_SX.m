function A = compute_A_SX(rotor_angles, alpha, arm, Kf, Km)
    addpath('../casadi_matlab')
    import casadi.*
    
    N_rotors = length(rotor_angles);
    rotor_eta = SX.sym('rotor_eta', 3, N_rotors);
    rotor_pos = SX.sym('rotor_pos',3, N_rotors);
    A = SX.sym('A',6, N_rotors);

    for i=1:N_rotors
        rotor_eta(:,i) = Rz(rotor_angles(i)) * Rx(alpha(i)) * [0;0;1];
        rotor_pos(:,i) = [cos(rotor_angles(i)); sin(rotor_angles(i)); 0] * arm;
    end
    
    
    for i=1:N_rotors
        % Force
        A(1:3,i) = Kf * rotor_eta(:,i);
    
        % Moment
        if( mod(i,2) == 0 )
            rotation = 1;
        else
            rotation = -1;
        end
        A(4:6,i) = skew(rotor_pos(:,i)) * Kf * rotor_eta(:,i) + rotation * Km * rotor_eta(:,i);
    
    end

end