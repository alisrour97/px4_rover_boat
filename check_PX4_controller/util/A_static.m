function A_st = A_static(rotor_angles, arm, Kf, Km)
    
    N_rotors = length(rotor_angles);
    rotor_eta = zeros(3, N_rotors);
    rotor_pos = zeros(3, N_rotors);
    A_ver = zeros(6, N_rotors);
    A_hor = zeros(6, N_rotors);
    A_st= [];

    % Vertical
    alpha = zeros(N_rotors,1);
    for i=1:N_rotors
    
        rotor_eta(:,i) = Rz(rotor_angles(i)) * Rx(alpha(i)) * [0;0;1];
        rotor_pos(:,i) = [cos(rotor_angles(i)); sin(rotor_angles(i)); 0] * arm;
    
        % Force
        A_ver(1:3,i) = rotor_eta(:,i);
    
        % Moment
        if( mod(i,2) == 0 )
            rotation = 1;
        else
            rotation = -1;
        end
        A_ver(4:6,i) = skew(rotor_pos(:,i)) * rotor_eta(:,i) + rotation * (Km/Kf) * rotor_eta(:,i);
    
    end
    
    % Horizontal
    alpha = ones(N_rotors,1)*pi/2;
    for i=1:N_rotors
    
        rotor_eta(:,i) = Rz(rotor_angles(i)) * Rx(alpha(i)) * [0;0;1];
        rotor_pos(:,i) = [cos(rotor_angles(i)); sin(rotor_angles(i)); 0] * arm;
    
        % Force
        A_hor(1:3,i) = rotor_eta(:,i);
    
        % Moment
        if( mod(i,2) == 0 )
            rotation = 1;
        else
            rotation = -1;
        end
        A_hor(4:6,i) = skew(rotor_pos(:,i)) * rotor_eta(:,i) + rotation * (Km/Kf) * rotor_eta(:,i);
    
    end
    
    
    for i=1:N_rotors
        A_st = [A_st, A_ver(:,i), A_hor(:,i)];
    end

end