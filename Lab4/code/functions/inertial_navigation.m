function varargout = inertial_navigation(varargin)
%INERTIAL_NAVIGATION is an intertial navigation solver for the Sensor
%   Orientation Lab 4. Before it can be used as part of a simulation loop, 
%   the function must be initialized.
%   Note: this function uses peristant variables, so only one instance of
%   INERTIAL_NAVIGATION may run at a time
%
% inertial_navigation('init')                          : to initialize this function with the bare minimum
% inertial_navigation('init', x0, dx0, ddx0, azth0, integration_method) : to fully initialize this function
%       available integration methods are: 'Euler', 'Trapezoid', 'Simpson'
%
% [x]                = inertial_navigation(fb, wb, dt) : to run this function as part of a loop. 
% [x, azth]          = inertial_navigation(fb, wb, dt)
% [x, dx, ddx, azth] = inertial_navigation(fb, wb, dt)

% $Author: Michael Biselx $     $Date: 30-10-2021$      $Revision: 0.0 $ 
% Copyright: no? 

    persistent xm vm am RotMat integrator

    if ischar(varargin{1}) || isstring(varargin{1})
        if strcmpi(varargin{1}, 'init')
            
            % initialize the position estimation
            if nargin > 1
                xm = varargin{2} .* ones(2, 3);
            else 
                xm = zeros(2,3);
            end
    
            % initialize the velocity estimation 
            if nargin > 2
                vm = varargin{3} .* ones(2, 3);
            else 
                vm = zeros(2,3);
            end
    
            % initialize the acceleration estimation
            if nargin > 3
                am = varargin{4} .* ones(2, 3);
            else 
                am = zeros(2,3);
            end
    
            % initialize the attitude estimation
            if nargin > 4
                RotMat = [ cos(varargin{5}) -sin(varargin{5})
                           sin(varargin{5})  cos(varargin{5})];
            else 
                RotMat = eye(2);
            end

            if nargin > 5
                switch varargin{6}
                    case 'Euler'
                        integrator = @(f, t) f(:,1)*t;
                    case 'Trapezoid'
                        integrator = @(f, t) .5*(f(:,1)+f(:,2))*t;
                    case 'Simpson' % this is garbage 
                        integrator = @(f, t) (f(:,1)+4*f(:,2)+f(:,3))*t/6;
                    otherwise
                        error("Unknown Integration Method '%s'", varargin{6})
                end 
            else 
                integrator = @(f, t) f(:,1)*t;
            end

            varargout = {true}; % initialization successful
        else 
            error("Unrecognised option '%s'. Type 'help inertial_navigation' for more info", varargin{1})
        end
        

    else
        if isempty(RotMat) || isempty(am) 
            error("The function must be initialized before it can be used! Type 'help inertial_navigation' for more info")
        end

        % step 1 : estimate the attitude from the gyro data
        RotMat = RotMat * expm([0 -varargin{2}; varargin{2} 0]*varargin{3});
        azth = atan2(-RotMat(1,2), RotMat(1,1));
    
        % step 2 : estimate the force in the map frame from accelerometer data
        fm = RotMat * varargin{1};
    
        % step 3 : convert force to acceleration
        am(:,length(am)) = fm;
        am = circshift(am,1,2);
    
        % step 4 : integrate to get velocity
        vm(:,length(vm)) = vm(:,1) + integrator(am, varargin{3}); 
        vm = circshift(vm,1,2);
    
        % step 5 : integrate to get position
        xm(:,length(xm)) = xm(:,1) + integrator(vm, varargin{3}); 
        xm = circshift(xm,1,2);
        
        if nargout == 1
            varargout = {xm(:,1)};
        elseif nargout < 3
            varargout = {xm(:,1), azth};
        else
            varargout = {xm(:,1), vm(:,1), am(:,1), azth};
        end
    end

end

