function rmse = CompareImages(I1, I2)
% Jianjiang Feng
% 2016-11-16
e = double(I1) - double(I2);
rmse = sqrt(sum(e(:).^2)/numel(e));