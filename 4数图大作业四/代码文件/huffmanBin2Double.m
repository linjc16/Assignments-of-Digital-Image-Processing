function x = huffmanBin2Double(y,bit_num)
% Jianjiang Feng
% 2016-11-16
x = dec2bin(y,8);
x = x';
x = double(x(1:bit_num))-48;
