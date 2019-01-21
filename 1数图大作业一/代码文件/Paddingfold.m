function [PadSrc1] = Paddingfold(ImgSrc1,fold)
%ImgSrc1为待padding图像
%fold为将图像的宽高padding成的倍数
[HeightSrc1,WidthSrc1] = size(ImgSrc1);
WidthPad1 = 0;
HeightPad1 = 0;
if(mod(WidthSrc1,fold)~=0)
    WidthPad1 = fold - mod(WidthSrc1,fold);
end
if(mod(HeightSrc1,fold)~=0)
    HeightPad1 = fold - mod(HeightSrc1,fold);
end
PadSrc1 = padarray(ImgSrc1,[HeightPad1,WidthPad1],'replicate','post');
end

