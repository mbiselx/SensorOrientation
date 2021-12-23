function [varargout] = simulate_IMU_noise(step, varargin)
%SIMULATE_IMU_NOISE simulates the noise of an IMU 
% simulate_IMU_noise('init', {gyro_wn, gyro_bias, [gyro_GM_sigma, gyro_GM_beta]})
% simulate_IMU_noise('init', {gyro_wn, {gyro_bias}, {gyro_GM}}, {ac1_wn, {ac1_GM}}, {ac2_wn, {ac2_GM}})
%
% simulate_IMU_noise('get', dt)

persistent gyro_wn gyro_bias gyro_GM_sigma gyro_GM_beta
persistent gyro_bias0 gyro_GM0
persistent ac1_wn ac1_GM_sigma ac1_GM_beta ac1_GM0 
persistent ac2_wn ac2_GM_sigma ac2_GM_beta ac2_GM0

    switch step
        case 'init'
            if nargin > 1       % gyro information is present
                if ~isempty(varargin{1}{1})
                    if iscell(varargin{1}{1}(1))
                        gyro_wn  = varargin{1}{1}{1};
                    else 
                        gyro_wn  = varargin{1}{1}(1);
                    end
                end
                if length(varargin{1}) > 1
                    if iscell(varargin{1}(2))
                        gyro_bias  = varargin{1}{2}{1};
                        gyro_bias0 = varargin{1}{2}{2};
                    else 
                        gyro_bias  = varargin{1}{2}(1);
                        gyro_bias0 = varargin{1}{2}(2);
                    end
                end
                if length(varargin{1}) > 2
                    if iscell(varargin{1}{3})
                        gyro_GM_sigma = varargin{1}{3}{1};
                        gyro_GM_beta  = varargin{1}{3}{2};
                        if length(varargin{1}{3})>2
                            gyro_GM0  = varargin{1}{3}{3};
                        end
                    else 
                        gyro_GM_sigma = varargin{1}{3}(1);
                        gyro_GM_beta  = varargin{1}{3}(2);
                        if length(varargin{1}{3})>2
                            gyro_GM0  = varargin{1}{3}(3);
                        end
                    end
                end
            end

            if nargin > 2       % accelerometer 1 information
                if ~isempty(varargin{2})
                    if iscell(varargin{2}{1})
                        ac1_wn     = varargin{2}{1}{1};
                    else 
                        ac1_wn     = varargin{2}{1};
                    end
                end
                if length(varargin{2}) > 1
                    if iscell(varargin{2}{2})
                        ac1_GM_sigma = varargin{2}{2}{1};
                        ac1_GM_beta  = varargin{2}{2}{2};
                        if length(varargin{2}{2})>2
                            ac1_GM0  = varargin{2}{2}{3};
                        end
                    else 
                        ac1_GM_sigma = varargin{2}{2}(1);
                        ac1_GM_beta  = varargin{2}{2}(2);
                        if length(varargin{2}{2})>2
                            ac1_GM0  = varargin{2}{2}(3);
                        end
                    end
                end
            end
    
            if nargin > 3       % accelerometer 2 information
                if ~isempty(varargin{3})
                    if iscell(varargin{3}{1})
                        ac2_wn     = varargin{3}{1}{1};
                    else 
                        ac2_wn     = varargin{3}{1}(1);
                    end
                end 
                if length(varargin{3}) > 1
                    if iscell(varargin{3}{2})
                        ac2_GM_sigma = varargin{3}{2}{1};
                        ac2_GM_beta  = varargin{3}{2}{2};
                        if length(varargin{3}{2})>2
                            ac2_GM0  = varargin{3}{2}{3};
                        end
                    else 
                        ac2_GM_sigma = varargin{3}{2}(1);
                        ac2_GM_beta  = varargin{3}{2}(2);
                        if length(varargin{3}{2})>2
                            ac2_GM0  = varargin{3}{2}(3);
                        end
                    end
                end
            end

        case 'get'
            dt = varargin{1};

            % gyro noise 
            gyro_noise = zeros(1,3);
            if ~isempty(gyro_wn)  
                gyro_noise(1) = wn_seq(gyro_wn, dt);
            end
            if ~isempty(gyro_bias) || ~isempty(gyro_bias0)
                if isempty(gyro_bias0)  % initialize it, in case it is not yet initialized 
                    gyro_bias0 = gyro_bias * randn();
                end
                gyro_noise(2) = gyro_bias0;
            end
            if ~isempty(gyro_GM_sigma) && ~isempty(gyro_GM_beta)
                gyro_GM0 = GM_seq(gyro_GM0, gyro_GM_sigma, gyro_GM_beta, dt);
                gyro_noise(3) = gyro_GM0;
            end
                
            % accelerometer noise 
            acc_noise = zeros(2,3);
            if ~isempty(ac1_wn) && ~isempty(ac2_wn)
                acc_noise(:,1) = wn_seq([ac1_wn; ac2_wn], dt);
            end
            if ~isempty(ac1_GM_sigma) && ~isempty(ac1_GM_beta)
                ac1_GM0 = GM_seq(ac1_GM0, ac1_GM_sigma, ac1_GM_beta, dt);
                acc_noise(1,3) = ac1_GM0; 
            end         
            if ~isempty(ac2_GM_sigma) && ~isempty(ac2_GM_beta)
                ac2_GM0 = GM_seq(ac2_GM0, ac2_GM_sigma, ac2_GM_beta, dt);
                acc_noise(2,3) = ac2_GM0; 
            end    

            if nargout <= 2
                varargout = {sum(gyro_noise,2), sum(acc_noise,2)};
            else 
                varargout = {sum(gyro_noise,2), sum(acc_noise,2), gyro_noise, acc_noise};
            end
    end
end






