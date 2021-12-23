function [varargout] = error_estimator(step, varargin)
%ERROR_HANDLER Summary of this function goes here
%   [gyr_err_hat, acc_err_hat] = error_estimator('update', dx2, betas, dt)
%   [gyr_err_hat, acc_err_hat] = error_estimator('get', dt)
    persistent beta 
    persistent d_wb b_gyr_c b_gyr_GM
    persistent d_fb b_acc_GM

    if isempty(d_wb) || isempty(d_fb)
        d_wb        = 0;
        b_gyr_c     = 0;
        b_gyr_GM    = 0;

        d_fb        = [0; 0];
        b_acc_GM    = [0; 0];
        beta        = zeros(3,1);
    end


    switch step
        case 'update' 
            beta        = varargin{2};
            dx          = varargin{1};
            
            b_gyr_c         = b_gyr_c + dx(1);
            b_gyr_GM        = b_gyr_GM *exp(-beta(1)  *varargin{3}) + dx(2);
            b_acc_GM        = b_acc_GM.*exp(-beta(2:3)*varargin{3}) + dx(3:4);
        
%             b_gyr_c         = b_gyr_c  + dx(1);
%             b_gyr_GM        = b_gyr_GM + dx(2);
%             b_acc_GM        = b_acc_GM + dx(3:4);
           
        case 'get' 
            

            gyr_err_hat = -b_gyr_GM *exp(-beta(1)  *varargin{1}) -b_gyr_c ;
            acc_err_hat = -b_acc_GM.*exp(-beta(2:3)*varargin{1});

%             gyr_err_hat = -b_gyr_c - b_gyr_GM;
%             acc_err_hat = -b_acc_GM;

            if nargout <= 2
                varargout = {gyr_err_hat, acc_err_hat};
            else
                varargout = {gyr_err_hat, acc_err_hat, ...
                    [0, b_gyr_c, b_gyr_GM *exp(-beta(1)  *varargin{1})], ...
                    [zeros(2),   b_acc_GM.*exp(-beta(2:3)*varargin{1})]};
            end

    end



end

