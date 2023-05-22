function eR = vmap(Rd,R)
% orientation error 
E = 0.5 * ((R'*Rd) - (Rd'*R));
%   
  
  
  eR =[(E(3, 2) - E(2, 3))/2;...
       (E(1, 3) - E(3, 1))/2;...
       (E(2, 1) - E(1, 2))/2];
%    
%  thrshld = 10^-4;  
%    
%  if abs(eR(1)) < thrshld
%      eR(1) = 0;
%  end
%  
%  if abs(eR(2)) < thrshld
%      eR(2) = 0;
%  end
%  
%  if abs(eR(3)) < thrshld
%      eR(3) = 0;
%  end
%      
% vector_1 = E(3,2);
% vector_2 = E(1,3);
% vector_3 = E(2,1);
% 
% eR = [ vector_1 ;
%            vector_2 ;
%            vector_3];

end

