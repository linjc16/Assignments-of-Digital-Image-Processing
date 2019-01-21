function [symbols, prob] = prob4huffman(x)
% Jianjiang Feng
% 2016-11-16
x = x(:);
symbols = unique(x);
[N,~] = histcounts(x,[symbols;symbols(end)+1]);
prob = N/numel(x);