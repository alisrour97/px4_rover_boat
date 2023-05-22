function R = quat2rotm(q)
    
    eta = q(1);
    vec = q(2:4);

    R = [2*(eta^2 + vec(1)^2)-1, 2*(vec(1)*vec(2) - eta*vec(3)), 2*(vec(1)*vec(3) + eta*vec(2));
         2*(vec(1)*vec(2) + eta*vec(3)), 2*(eta^2 + vec(2)^2)-1, 2*(vec(2)*vec(3) - eta*vec(1));
         2*(vec(1)*vec(3) - eta*vec(2)), 2*(vec(2)*vec(3) + eta*vec(1)), 2*(eta^2 + vec(3)^2)-1];

end