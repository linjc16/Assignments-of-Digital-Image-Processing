function Fingerprint2(ImgSrc1)
%%
%读取图片
ImgSrc1 = 1 - im2double(ImgSrc1);
%%
%将指纹图像分为许多8×8像素的图像块。对每个图像块计算DFT
%先padding至8的倍数
PadSrc1 = Paddingfold(ImgSrc1,8);
%%
%粗分割出前背景
%先扩充
blksize = 8;
threshold = 0.01;
PadSrc2 = Paddingfold(ImgSrc1,blksize);
imsegment = zeros(size(PadSrc2));
immask = zeros(size(PadSrc2));
for i = 1:blksize:size(PadSrc2,1)
    for j = 1:blksize:size(PadSrc2,2)
        V = var(PadSrc2(i:i+blksize-1,j:j+blksize-1));
        if V < threshold
            imsegment(i:i+blksize-1,j:j+blksize-1) = 1;
        else
            immask(i:i+blksize-1,j:j+blksize-1) = 1;
        end
    end
end
immask = imfill(immask,'holes');
immask = ~imfill(~immask,'holes');
PadSrc1 = PadSrc1 .* immask;
%%
%由于需要扩大至32*32，所以周围需要padding 12
PadPadSrc1 = padarray(PadSrc1,[12,12],'replicate','both');
%将每块(32*32)进行DFT变换
[HeightPad1,WidthPad1] = size(PadSrc1);
wi = WidthPad1 / 8;
hi = HeightPad1 / 8;
DftSrc = zeros(32*hi,32*wi);
for i = 1 : wi
    for j = 1 : hi
        hmin = 8 * (j - 1) + 1;
        hmax = hmin + 32 - 1;
        wmin = 8 * (i - 1) + 1;
        wmax = wmin + 32 - 1;
        DftPatch = PadPadSrc1(hmin:hmax,wmin:wmax);
        DftPatch(17,17) = 0;
        hminDft = 32 * (j - 1) + 1;
        hmaxDft = hminDft + 32 - 1;
        wminDft = 32 * (i - 1) + 1;
        wmaxDft = wminDft + 32 - 1;
        DftPatch = fftshift(fft2(DftPatch));
        DftPatch(17,17) = 0;
        DftSrc(hminDft:hmaxDft,wminDft:wmaxDft) = DftPatch;
    end
end
%归一化
DftSrcNorm = (DftSrc - min(min(DftSrc)))/ (max(max(DftSrc))-min(min(DftSrc)));
%%
%根据幅度谱，估计每个图像块是否属于指纹区域；如果属于指纹区域，估计脊线方向和频率。 
OrienField = zeros(size(PadSrc1));
OrienPatch = zeros(8,8);
OrienAngle = zeros(hi,wi);
OrienWavelen = zeros(hi,wi);
OrienGraph = zeros(hi,wi);
ImMask = ones(size(PadSrc1));
for i =  1 : wi
    for j = 1 : hi
        %读取幅度图中的两个最大值 32*32 每块
        hminDft = 32 * (j - 1) + 1;
        hmaxDft = hminDft + 32 - 1;
        wminDft = 32 * (i - 1) + 1;
        wmaxDft = wminDft + 32 - 1;
        Patch = DftSrcNorm(hminDft:hmaxDft,wminDft:wmaxDft);
        [Peak,indices] = maxk(abs(Patch(:)),2);
        PeakX1 = ceil((indices(1) - 1)/32);
        PeakY1 = mod((indices(1) - 1),32) + 1;
        PeakX2 = ceil((indices(2)-1)/32);
        PeakY2 = mod((indices(2)-1),32) + 1;
        %计算方向场角度
        theta = -atan((PeakY1(1)-PeakY2(1)) / (PeakX1(1)-PeakX2(1)));
        %计算低频部分占比
        PatchLow = sum(sum(abs(Patch(16:18,16:18))));
        PatchLF = PatchLow / sum(sum(abs(Patch)));
        %计算高频部分幅度占比
        PatchH = sum(sum(abs(Patch(10:24,10:24))));
        PatchL = sum(sum(abs(Patch(15:19,15:19))));
        PatchHF = (PatchH - PatchL) / (sum(sum(abs(Patch))));
        if(PatchHF < 0.33 || PatchLF > 0.124)
            hmin = 8 * (j - 1) + 1;
            hmax = hmin + 8 - 1;
            wmin = 8 * (i - 1) + 1;
            wmax = wmin + 8 - 1;
            ImMask(hmin:hmax,wmin:wmax) = 0;
            continue;
        end
        CheckX = PeakX1(1);
        CheckY = PeakY1(1);
        if((abs(CheckX - 17) < 5  || abs(CheckY - 17) < 5) && (Peak(1) > 0.6 || Peak(1) < 0.115))
            hmin = 8 * (j - 1) + 1;
            hmax = hmin + 8 - 1;
            wmin = 8 * (i - 1) + 1;
            wmax = wmin + 8 - 1;
            ImMask(hmin:hmax,wmin:wmax) = 0;
            continue;
        end
        %计算wavelength
        OrienWavelen(j,i) = 64 / sqrt((PeakY1(1)-PeakY2(1))^2+(PeakX1(1)-PeakX2(1))^2);
        angleOrien = 180 - (theta(1) / pi) * 180;
        OrienGraph(j,i) = angleOrien;
        OrienAngle(j,i) = theta(1) / pi * 180;
        OrienPatch(:,:) = 0;
        OrienPatch(5,3:6) = 1;
        OrienPatchRot = imrotate(OrienPatch,angleOrien,'bilinear','crop');
        %赋值给新的方向图 8*8 每块
        hmin = 8 * (j - 1) + 1;
        hmax = hmin + 8 - 1;
        wmin = 8 * (i - 1) + 1;
        wmax = wmin + 8 - 1;
        %旋转后对中间部分进行选取
        [hOri,wOri] = size(OrienPatchRot);
        hminOri = floor(hOri / 2) + 1 - 4;
        hmaxOri = hminOri + 8 - 1;
        wminOri = floor(wOri / 2) + 1 - 4;
        wmaxOri = wminOri + 8 - 1;
        OrienField(hmin:hmax,wmin:wmax) = OrienPatchRot(hminOri:hmaxOri,wminOri:wmaxOri);
    end
end
PadSrcHole = PadSrc1 .* imfill(ImMask,'holes');
PadSrcHole = imbinarize(PadSrcHole,0.01);
PadSrcHole = imfill(PadSrcHole,'holes');
PadSrcHole = ~imfill(~PadSrcHole,'holes');
PadSrc1 = PadSrc1 .* PadSrcHole;
figure(1);
imshow(PadSrc1 .* PadSrcHole);

figure(2);
subplot(121);
quiver(flip(cos(-pi / 180 .* OrienAngle)), flip(sin(-pi / 180 .* OrienAngle)));
title("平滑之前的方向图");
subplot(122);
imshow(2.*pi./OrienWavelen,[]);
title("平滑之前的频率图");
%%
%由于噪声干扰，某些块的方向和频率可能是错误的。利用空域平滑滤波方法，对方向图、频率图分别进行平滑。
w2 = fspecial('gaussian',3,0.5) ;
OrienWavelen = imfilter(OrienWavelen, w2);
g =   cos((2 / 180 * pi) .* OrienAngle)+ 1i*sin((2 / 180 * pi) .* OrienAngle);
g =   imfilter(g,w2);
OrienAngle = 0.5 .* angle(g) ./ pi .* 180;
%显示
figure(3);
subplot(121);
quiver(flip(cos(-pi / 180 .* OrienAngle)), flip(sin(-pi / 180 .* OrienAngle)));
title("平滑之后的方向图");
subplot(122);
imshow(2.*pi./OrienWavelen,[]);
title("平滑之后的频率图");
%%
%利用空域 Gabor 滤波器对指纹进行滤波，得到增强图。每个像素的 Gabor 滤波器参数由方向图和频率图决定
PadPadSrc2 = padarray(PadSrc1,[4,4],'replicate','both');
ImgOutput = zeros(size(PadPadSrc2));
for i =  1 : wi
    for j = 1 : hi
        if(OrienWavelen(j,i) <=2)
             continue;
        end
        %对每块进行Gabor滤波，16*16 每块
        hmin = 8 * (j - 1) + 1;
        hmax = hmin + 16 - 1;
        wmin = 8 * (i - 1) + 1;
        wmax = wmin + 16 - 1;
        [mag,phase] = imgaborfilt(PadPadSrc2(hmin:hmax,wmin:wmax),OrienWavelen(j,i),OrienAngle(j,i));
        OutTmp = mag .*cos(phase);
        %将每一块进行拼接
        ImgOutput(hmin:hmax,wmin:wmax) = ImgOutput(hmin:hmax,wmin:wmax) + OutTmp;
    end
end
final_image = imbinarize(ImgOutput,0.95);
final_image=bwmorph(final_image,'thin','inf');
se = strel('sphere',2);
final_image=imdilate(final_image,se);


figure(1);
f1 = subplot(231);imshow(ImgSrc1);title("输入图像");
subplot(232);imshow(PadSrcHole);title('mask');
subplot(233);imshow(abs(DftSrcNorm));title("DFT幅度谱");
subplot(234);imshow(OrienField);title("简易方向场");
f2 = subplot(235);imshow(ImgOutput);title("脊线增强后的结果");
f3 = subplot(236);imshow(final_image);title("二值化及细化后的结果");
linkaxes([f1,f2,f3]);
end

