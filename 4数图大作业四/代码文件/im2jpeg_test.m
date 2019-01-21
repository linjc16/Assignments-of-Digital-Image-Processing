% im2jpeg_test
%
% Jianjiang Feng
% 2016-11-16

f = imread('.\data\1_1.bmp');
% f = imread('..\data\Fig0809(a).tif');

c1 = im2jpeg(f, 1);
f1 = jpeg2im(c1);
cr1 = imratio(f, c1)
rmse1 = CompareImages(f, f1)

c4 = im2jpeg(f, 4);
f4 = jpeg2im(c4);
cr4 = imratio(f, c4)
rmse4 = CompareImages(f, f4)

%close all
figure(1),
ax(1)=subplot(1,3,1);imshow(f);
ax(2)=subplot(1,3,2);imshow(f1);
ax(3)=subplot(1,3,3);imshow(f4);
linkaxes(ax);

figure(2),
ax2(1)=subplot(1,3,1);imshow(f);
ax2(2)=subplot(1,3,2);imshow(double(f1)-double(f),[]);
ax2(3)=subplot(1,3,3);imshow(double(f4)-double(f),[]);
linkaxes(ax2);

figure(3),
ax3(1)=subplot(1,3,1);imshow(f);
ax3(2)=subplot(1,3,2);imshow(f4);
ax3(3)=subplot(1,3,3);imshow(double(f4)-double(f),[]);
linkaxes(ax3);
