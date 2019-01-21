close all;
clear;
clc;
videoname = './videos/8.mp4';
obj = VideoReader(videoname);
obj.CurrentTime = 0;
framenum = 0;
frameaver = zeros(obj.Height,obj.Width,3);
%存储所有帧并求取平均以得到静态背景参考图
while obj.CurrentTime < obj.Duration
    framenum = framenum + 1;
    frame{framenum,1} = im2double(readFrame(obj));
    frameaver = frameaver + frame{framenum,1};
end
frameaver = frameaver / framenum;
% figure;
% imshow(frameaver);
%% 
firstframe = frame{1,1};
figure,imshow(firstframe);
title('点选球的附近并回车')
[x,y] = initial();
% [xlast,ylast]记录上一次的中心位置
% [xcurr,ycurr]记录当前的中心位置
xlast = x;
ylast = y;
xcurr = x;
ycurr = y;
%deltax、deltay记录两帧间球的位移量
deltax = 0;
deltay = 0;
% 记录下初始坐标并存入lines中
count = 1;
%记录下中心点
centrid = [];
lines = [];
lines(count,:) = [x,y];
% 记录下球的半径大小
radius = [];

figure;
for i = 1:framenum
    %分割
    if i == 1
       xpredict = xlast;
       ypredict = ylast;
    else
        xpredict = xcurr + xcurr - xlast;
        ypredict = ycurr + ycurr - ylast;
    end
    ballground = frame{i,1} - frameaver;
%     ballground = rgb2gray(ballground);
%     ballpic = frame{i,1}(ypredict-25:ypredict+25,xpredict-25:xpredict+25);
%     ballpicrsz = imresize(ballpic,[size(ballpic,1)*3,size(ballpic,2)*3]);
     Rectlen = 20;
     fold = 3;
     MiniBatchCenter = floor(((Rectlen * 2 + 1) * fold - 1) / 2);
     
     ballpic = ballground(ypredict-Rectlen:ypredict+Rectlen,xpredict-Rectlen:xpredict+Rectlen);
     ballpicrsz = imresize(ballpic,[size(ballpic,1)*fold,size(ballpic,2)*fold]);
%     imshow(ballpic);
%     imshow(ballpicrsz);
    imshow(frame{i,1});
    [centersDk,radiiDk] = imfindcircles(ballpicrsz,[6 20],'ObjectPolarity','dark');
    [centersBgt,radiiBgt] = imfindcircles(ballpicrsz,[6 20],'ObjectPolarity','bright');
%     viscircles(centersDk,radiiDk,'EdgeColor','b');
%     viscircles(centersBgt,radiiBgt,'EdgeColor','r');
    centerball = [];
    radiusball = [];
    if ~isempty(centersDk)
        centerball_X =  (centersDk(:,1) <  MiniBatchCenter+ 27)  & (centersDk(:,1)  >  MiniBatchCenter - 27);
        centerball_Y =  (centersDk(:,2) <  MiniBatchCenter + 27) & (centersDk(:,2)  >  MiniBatchCenter - 27);
        centerballrow = find((centerball_X & centerball_Y) == 1);
        if isempty(centerballrow)
            continue;
        end
        %记录中心
        centerball(1,1) = centersDk(centerballrow(1,1),1);
        centerball(1,2) = centersDk(centerballrow(1,1),2);
        %记录半径
        radiusball(1,1) = radiiDk(centerballrow(1,1),1);
    else
        if isempty(centersBgt)
            continue;
        end
        centerball_X =  (centersBgt(:,1) < MiniBatchCenter + 27) & (centersBgt(:,1) > MiniBatchCenter - 27);
        centerball_Y =  (centersBgt(:,2) < MiniBatchCenter + 27) & (centersBgt(:,2) > MiniBatchCenter - 27);
        centerballrow = find((centerball_X & centerball_Y) == 1);
        if isempty(centerballrow)
            continue;
        end
        %记录中心
        centerball(1,1) = centersBgt(1,1);
        centerball(1,2) = centersBgt(1,2);
        %记录半径
        radiusball(1,1) = radiiBgt(1,1);
    end
    
    if isempty(centerball)
        xlast = xcurr;
        ylast = ycurr;
        xcurr = xpredict + deltax * 0.4;
        ycurr = ypredict + deltay * 0.4;
        count = count+1;
        hold on
        lines(count,:) = [xcurr,ycurr];
        plot(lines(:,1),lines(:,2),'r-');
        hold off
        continue;
    else
        xlast = xcurr;
        ylast = ycurr;
        deltax = floor((centerball(1,1) - MiniBatchCenter) / 3) / 4;
        deltay = floor((centerball(1,2) - MiniBatchCenter) / 3) / 4;
        xcurr = floor((centerball(1,1) - MiniBatchCenter) / 3) / 4  + xpredict;
        ycurr = floor((centerball(1,2) - MiniBatchCenter) / 3) / 4  + ypredict;
        radius(count,1) = radiusball(1,1) / 3;
        centrid(count,:) = [floor((centerball(1,1) - MiniBatchCenter) / 3) + xpredict ,...
            floor((centerball(1,2) - MiniBatchCenter) / 3) + ypredict];
        count = count + 1;
        hold on
        lines(count,:) = [floor((centerball(1,1) - MiniBatchCenter) / 3) + xpredict , ...
            floor((centerball(1,2) - MiniBatchCenter) / 3) + ypredict];
        plot(lines(:,1),lines(:,2),'r-');
        hold off
        viscircles([floor((centerball(1,1) - MiniBatchCenter) / 3) + xpredict ,...
            floor((centerball(1,2) - MiniBatchCenter) / 3) + ypredict],radiusball / 3,'EdgeColor','b');
    end
    
    pause(0.001);
end
%% 计算速度(普通)
%计算球的半径
radius_4 = 0.19 / 2 ; %4号足球直径19cm
radius_5 = 0.215 / 2; %5号足球直径21.5cm
radius_pri = radius_5;
Startid = 7;
Endid = size(radius,1) - 5;
r_mean = sum(radius(Startid:Endid,1)) / (Endid - Startid);
ballshift  = [];
vball = [];
shiftcount = 0;
vballcount = 0;
for i = Startid:Endid-1
    shiftcount = shiftcount + 1;
    ballshift(shiftcount,1) = caldistance(centrid(i,1),centrid(i,2),centrid(i+1,1),centrid(i+1,2));
end
for i = Startid + 1:Endid - 1
    vballcount = vballcount + 1;
    shift = caldistance(centrid(i-1,1),centrid(i-1,2),centrid(i+1,1),centrid(i+1,2));
    vball(vballcount,1) = shift / 2;
end
v_mean = sum(ballshift) / shiftcount(1,1);
v_mean = v_mean / r_mean * radius_pri * 240;
vball = vball / r_mean * radius_pri * 240;
%% 计算速度(指数移动平均)
alpha  = 0.5;
v_mean_EMA = alpha * vball(1,1);
for i = 2:vballcount - 10
    if(vball(i,1) - v_mean_EMA > 10 || vball(i,1) - v_mean_EMA < -10 || vball(i,1) > 35)
        continue;
    end
    v_mean_EMA = alpha * vball(i,1) + (1 - alpha) * v_mean_EMA;
end
% 计算最大速度
v_max = max(vball(vball<30));
%% 一些函数  
    function [x,y] = initial()
        [x,y] = ginput(1);
        x = int16(x);
        y = int16(y);
    end
    
    function [distance] = caldistance(x1,y1,x2,y2)
        distance = ((x2 - x1)^2 + (y2 - y1)^2) ^(0.5);
    end
