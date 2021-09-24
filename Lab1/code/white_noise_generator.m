function wn = white_noise_generator(k, sigma=1)
% Creates a zero-mean white noise series of a given length k and a variance sigma

    wn = sigma*randn(1,k);

end
