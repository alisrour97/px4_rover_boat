function RPY = rotm2eul(R)
    
    yaw = atan2(R(2,1),R(1,1));
    pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    roll = atan2(R(3,2),R(3,3));

    RPY = [roll; pitch; yaw];

end