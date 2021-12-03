function fig = plot_everything(varargin)
%PLOT_EVERYTHING plots everything in the simulation for the Sensor
%   Orientation Lab 4. It can be used with different combinatins of input 
%   arguments, however they must be arranged in cells as shown below.
%   PLOT_EVERYTHING returns a handle to the figure used for plots, for making GIFs
%
% [FigureHandle] = plot_everything({trajectory})
% [FigureHandle] = plot_everything({trajectory, true_pose})
% [FigureHandle] = plot_everything({trajectory, true_pose, estimated_pose})
% [FigureHandle] = plot_everything({trajectory, true_pose}, {t, fb}, figN)
% [FigureHandle] = plot_everything({trajectory, true_pose, estimated_pose}, {t, fb1, fb2}, figN)
% [FigureHandle] = plot_everything({trajectory, true_pose}, {t, fb, wb}, figN)
% [FigureHandle] = plot_everything({trajectory, true_pose, estimated_pose, estd_traj}, {t, fb_nominal, wb_nominal, fb, wb}, figN)
% [FigureHandle] = plot_everything({trajectory, true_pose, estimated_pose, estd_traj, cov}, {t, fb_nominal, wb_nominal, fb, wb}, figN)


% $Author: Michael Biselx $     $Date: 30-10-2021$      $Revision: 0.0 $ 
% Copyright: no? 


    if nargin > 2
        fig = figure(varargin{3});
    else
        fig = figure(100);
    end
    fig.Name = "Simulation";

    if nargin > 1
        nb_sensors = min(length(varargin{2}), 3);
        idx = reshape(1:(2*nb_sensors), 2,[]);

        subplot(nb_sensors, 2, idx(1,:)) % plot trajectory
    end 

    h = plot(varargin{1}{1}(2,:), varargin{1}{1}(1,:), '--b'); hold on
    labels = {'trajectory'};
    if length(varargin{1}) > 1 && ~isempty(varargin{1}{2})
        h = [h, plot(varargin{1}{2}(2), varargin{1}{2}(1), 'bo', 'MarkerSize', 5.5)];
        labels = [labels, 'true position'];
    end
    if length(varargin{1}) > 3 && ~isempty(varargin{1}{4})
        h = [h, plot(varargin{1}{4}(2,:), varargin{1}{4}(1,:), '--r')];
        labels = [labels, 'est. trajectory'];
    end
    if length(varargin{1}) > 2 && ~isempty(varargin{1}{3})
        h = [h, plot(varargin{1}{3}(2), varargin{1}{3}(1), 'ro', 'MarkerSize', 6.0)];
        labels = [labels,'est. position'];
    end
    if length(varargin{1}) > 4 && ~isempty(varargin{1}{5})
        h = [h, plot_uncertainty([varargin{1}{3}(2);varargin{1}{3}(1)], ...
                                  [0 1; 1 0]*varargin{1}{5}*[0 1; 1 0],...
                                  '-.r')];
        labels = [labels,'3\sigma uncert.'];
    end
    if length(varargin{1}) > 5 && ~isempty(varargin{1}{6})
        h = [h, scatter(varargin{1}{6}(2,:), varargin{1}{6}(1,:), 'xk')];
        labels = [labels,'GPS.'];
    end

    hold off;
    axis equal 
    xlabel('E [m]'), ylabel('N [m]')
    title('Trajectory in Mapping Frame')
     
    if nargin == 1
        legend(h, labels, 'Location', 'southwest')
    else 
        legend(h, labels, 'Location', 'south')
        subplot(nb_sensors,2, 2)
            if length(varargin{2}) > 3
                hold on
                plot(varargin{2}{1}, varargin{2}{4}(1,:), 'r', 'LineWidth', .5) % plot f1_measured
            end
            plot(varargin{2}{1}, varargin{2}{2}(1,:), 'b', 'LineWidth', 1.2) % plot f1_nominal
            hold off
            grid on
            axis tight
            xlabel('t [s]'), ylabel('f_1^b [m/s^2]')
            title("Sensor Values")
        subplot(nb_sensors,2, 4)
            if length(varargin{2}) > 3
                hold on
                plot(varargin{2}{1}, varargin{2}{4}(2,:), 'r', 'LineWidth', .5) % plot f2_measured
            end
            plot(varargin{2}{1}, varargin{2}{2}(2,:), 'b', 'LineWidth', 1.2) % plot f2_nominal
            hold off
            grid on
            axis tight
            xlabel('t [s]'), ylabel('f_2^b [m/s^2]')
        if nb_sensors > 2
        subplot(nb_sensors,2,6)
            if length(varargin{2}) > 4
                hold on
                plot(varargin{2}{1}, varargin{2}{5}, 'r', 'LineWidth', .5)
            end
            plot(varargin{2}{1}, varargin{2}{3}, 'b', 'LineWidth', 1.2)
            hold off
            grid on
            axis tight
            xlabel('t [s]'), ylabel('\omega^b [°/s]')
        end

    end

    drawnow

end

