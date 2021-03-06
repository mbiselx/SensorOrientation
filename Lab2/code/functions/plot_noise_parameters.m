function plot_noise_parameters(varargin)
% function plot_noise_parameters(time, {x}, title, plt_option, nfig)
% function plot_noise_parameters(time, {x, autocorrelation, PSD}, title, plt_option, nfig)
% function plot_noise_parameters(time, {x, autocorrelation, PSD, allanvar}, title, plt_option, nfig)
%
% plots a noise time series, as well as any noise characteristics that are
% given.
% time      : time-vector
% x         : noise timeseries
% autocorr. : autocorrelation 
% PSD       : power spectral density. must be real values
% allanvar  : Allan variance of the noise
% title     : title to display over the plot
% plt_option: plot options 
% nfig      : name of the figure to plot this in (if blanc -> new figure)


    ylbls = {'signal', 'autocorr.', 'PSD', 'Allan var'};
    
    if(nargin >= 4)
        plt_option = varargin{4};
    else
        plt_option = '-';
    end

    if(nargin >=5)
        figure(varargin{5});
    else 
        figure();
    end

    n_sbplt = min(length(varargin{2}), 4);
    
    for i = 1:n_sbplt
        if (n_sbplt <= 3)
            subplot(n_sbplt, 1, i);
        else
            subplot(ceil(n_sbplt/2), 2, i);
        end
        switch (i)
            case 1    % time series
                plot(varargin{1}, varargin{2}{i}, plt_option)
                xlabel("t [s]"); 
            case 2   % autocorrelation
                dt = mean(diff(varargin{1}));
                plot(dt*((1:length(varargin{2}{i}))-(length(varargin{2}{i})/2-1)), varargin{2}{i}, plt_option);
                xlabel('\tau [s]')
            case 3   % PSD
                f  = ((1:length(varargin{2}{i}))-(length(varargin{2}{i})/2-1))./dt/length(varargin{2}{i});
                semilogy(f, varargin{2}{i}, plt_option);
                xlabel('f [Hz]')
                grid on
            case 4   % allen variance
                loglog(varargin{2}{i}(:,1), varargin{2}{i}(:,2), [plt_option(1), '-']) 
                xlabel('\tau [s]')
                grid on
        end
      
        ylabel(ylbls{i})
        axis('tight')
        if i == 3
            ylim([5e-8, 1e-2])
        end
        
        hold on
    end
    if nargin >= 3
        sgtitle(varargin{3})
    end 

end
