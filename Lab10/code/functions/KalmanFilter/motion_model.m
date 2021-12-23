function [F, G] = motion_model(x, fb, betas)
%MOTION_MODEL Summary of this function goes here
%   [F, G] = motion_model(x, fb)
%   [F, G] = motion_model(x, fb, betas)

    % navigation states : x_dot = F*x1 + G*w
    F           = zeros(5);
    F(2:3,1)    = RotMat(-x(1))*[0,-1;1,0]*fb;
    F(4:5,2:3)  = eye(2);
     
    G           = [RotMat(-x(1),1); zeros(2,3)];
    
    % augmentation : x_dot = F*x + G*w , x = [x1; x2]
    if exist("betas", 'var') && ~isempty(betas)
        F12     = [[1;zeros(4,1)], G];
        F21     = zeros(4, 5);
        F22     = diag([0; -betas]);
            
        G12     = zeros(5,3);
        G21     = zeros(4,3);
        G22     = [zeros(1,3); eye(3)];
            
        F       = [F, F12; F21, F22];
        G       = [G, G12; G21, G22];
    end 

end

