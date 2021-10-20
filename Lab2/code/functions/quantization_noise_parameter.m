function QNT = quantization_noise_parameter(x)
% qn = quantization_noise_parameter(x)
% find the quantization parameter of a singal

    QNT = mean(diff(unique(x)));

end