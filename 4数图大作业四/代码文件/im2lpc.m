function Y = im2lpc(X, f)
%IM2LPC ¶ÔÍ¼Ïñ×ö1-D linear predictive coding
% 2016-11-16
% reference: mat2lpc.m in DIPUM

if nargin<2
    f = 1; % default: previous pixel coding
end

X = double(X);
[M, N] = size(X);
P = zeros(M,N);

Xs = X;
for j = 1:length(f)
    Xs = [zeros(M,1) Xs(:,1:end-1)];
    P = P + f(j) * Xs;
end
Y = X - round(P);