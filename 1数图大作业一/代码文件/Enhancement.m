function Enhancement(mode)

if mode == 1
    ImgSrc1 = imread('FTIR.bmp');
    Fingerprint(ImgSrc1);
end
if mode == 3
    ImgSrc1 = imread('phone.bmp');
    Fingerprint(ImgSrc1);
end
if mode == 2
    ImgSrc1 = imread('latent.bmp');
    Fingerprint2(ImgSrc1);
end

end

