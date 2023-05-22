function A = compute_A_sim(rotor_angles, alpha, arm, Kf, Km)
    
    N_rotors = length(rotor_angles);
    rotor_eta = sym('rotor_eta', [3, N_rotors],'real');
    rotor_pos = sym('rotor_pos', [3, N_rotors],'real');
    A = sym('A', [6, N_rotors],'real');

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
        A(4:6,i) = skewMat(rotor_pos(:,i)) * Kf * rotor_eta(:,i) + rotation * Km * rotor_eta(:,i);
    
    end

end