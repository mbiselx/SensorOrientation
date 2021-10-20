function [PER] = PeriodicProcess(k, idx, A)
% [pp] = PeriodicProcess(k, idx, A)

    F_PER = zeros(k, size(idx,2));
    for i = 1:size(idx,2)
        F_PER(idx(:,i)-1, i) = pi*sqrt(8*k*A(:,i));
    end
    F_PER = [F_PER; zeros(1,size(F_PER,2)); flip(F_PER(2:end,:),1)]; % create a symmetrical dft, so that the output is real
    PER   = ifft(F_PER, [], 1);
    PER   = PER(floor(k/2):(floor(k/2)+k-1), :); % take central bit

end