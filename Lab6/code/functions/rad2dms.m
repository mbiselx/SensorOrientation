function [varargout] = rad2dms(alpha_in)
%RAD2DMS Summary of this function goes here
%   Detailed explanation goes here

    alpha_in = rad2deg(alpha_in);

    alpha_d = fix( alpha_in);
    alpha_m = fix((alpha_in - alpha_d) * 60);
    alpha_s =    ((alpha_in - alpha_d) * 60 - alpha_m) * 60;    
    
    if nargout < 3
        varargout = {[alpha_d, alpha_m, alpha_s]};
    else 
        varargout = { alpha_d, alpha_m, alpha_s };
    end
end

