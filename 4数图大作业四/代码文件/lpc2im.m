function x = lpc2im(y, f)
%lpc2im Decode a 1-D linear predictive encoded image
% 2016-11-17
% reference: lpc2mat.m in DIPUM

if nargin<2
    f = 1; % default: previous pixel coding
end

f = f(end:-1:1);
[m, n] = size(y);
order = length(f);
f = repmat(f, m, 1);
x = zeros(m, n + order);

for j = 1:n
    jj = j + order;
    x(:, jj) = y(:, j) + round(sum(f(:, order:-1:1).*x(:, (jj-1):-1:(jj-order)),2));
end
x = x(:, order+1:end);