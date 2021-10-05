function plot_noise(x, x_ac, x_psd, tit, N=[])

if isempty(N)
    figure()
else
    figure(N)
end

subplot(3,1,1)
    title(tit)
    plot(x);
    hold on
    xlabel("t [s]")
    ylabel("cnt [a.u.]")
subplot(3,1,2)
    plot((1:length(x_ac))-length(x_ac)/2, real(x_ac));
    hold on
    xlabel('\tau [s]')
    ylabel("autocorrelation [a.u.]")
subplot(3,1,3)
    plot((1:length(x_ac))-length(x_ac)/2, real(x_psd));
    hold on
    xlabel("f")
    ylabel("PSD [a.u.]")


end
