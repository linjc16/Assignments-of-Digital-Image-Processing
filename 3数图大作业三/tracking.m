function tracking()
% ���ǻ�������Ĵ����ܣ�����Բο�����������ʵ������Ķ�λ��׷��
% ��ֻ�����initial(), segment(), calcenter()��calspeed()�����������Ϳ���ʵ�ֻ��������Ҫ��
% ��Ҫ˵�����ǣ�Ϊ���ô������³���ԣ���Ӧ���������������Ƶ��������Ҫ�˽�������ܲ�����������
% ��������Ĳ������ٶȵļ��㲿�����������
% 
% ����ʾ������ȥ��ע�ͺ�����Ϳ���������
% ����ʾ������ľ��Ⱥ�³���Զ�û���Ż���
% �����κ����ʣ�����ʱ��ϵ�ң�������һ�𹹽����磡
%
% ��չ��
% 2018.12.5

% �޸Ĳ��ŵ�idֵ
vedioid = 8;
videoname = sprintf('./videos/%d.mp4',vedioid);
% ѡȡ��Ӧ�İ뾶
radius_4 = 0.19 / 2 ; %4������ֱ��19cm
radius_5 = 0.215 / 2; %5������ֱ��21.5cm
if(vedioid == 0)
    radius_pri = radius_4;
else
    radius_pri = radius_5;
end
obj = VideoReader(videoname);
firstframe = readFrame(obj);
figure,imshow(firstframe);
title('��ѡ������ĵ㲢�س�')
[x,y] = initial();
%%
% ��¼�³�ʼ���겢����lines��
count = 1;
lines = [];
centrid = [];
lines(count,:) = [x,y];
radius = [];
%%
frame = readFrame(obj);
while obj.CurrentTime<obj.Duration
%   �Զ����ÿ֡���д��������һ֡���������꣨x,y�����õ�����ķָ�ǰ��ͼ
    ballground = segment(frame,firstframe,x,y);
    imshow(frame)
%   ��������ǰ��ͼȷ�����������
    [x,y,area] = calcenter(ballground);
    centrid(count,:) = [x,y];
    radius(count,1) = sqrt(area / pi);
    count = count+1;
    if area < 1
        p = lines(count - 1,:);
        pp = lines(count - 2,:);
        x = p(1) + (p(1) - pp(1));
        y = p(2) + (p(2) - pp(2));
        hold on
        lines(count,:) = [x,y];
        plot(lines(:,1),lines(:,2),'r-');
        hold off
        pause(0.001);
        firstframe = frame;
        frame = readFrame(obj);
        continue;
    end
%   ��¼���������lines�У�����ͼ
%   ��¼����һ֡������ǰ��ͼ��������segs��
    hold on
    lines(count,:) = [x,y];
    plot(lines(:,1),lines(:,2),'r-');
    pause(0.001);
    hold off
%   ����������һ֡��ֱ������
    firstframe = frame;
    frame = readFrame(obj);
end
%%
%   ������ÿһ֡�󣬸��ݱ��������ָ�ͼ��segs�����һЩ����֪ʶ����������������������ٵ�
%������İ뾶
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
%% �����ٶ�(ָ���ƶ�ƽ��)
alpha  = 0.5;
v_mean_EMA = alpha * vball(1,1);
for i = 2:vballcount - 10
    if(vball(i,1) - v_mean_EMA > 10 || vball(i,1) - v_mean_EMA < -10 || vball(i,1) > 35)
        continue;
    end
    v_mean_EMA = alpha * vball(i,1) + (1 - alpha) * v_mean_EMA;
end
% ��������ٶ�
v_max = max(vball(vball<30));
%%
% 
%   ����������չʾ�����Ĵ��룬�ɺ���
%     h = 20;
%     w = 20;
%     bw2 = rgb2gray(frame);
%     mask = zeros(size(bw2));
%     mask(y-h:y+h,x-w:x+w) = 1;
%     ballground = mask&((bw2>200)|abs(bw2-bw)>20);
%     B = ones(3);
%     ballground = imerode(ballground,B);
%     ballground = imdilate(ballground,B);
%     for i = 1:3
%         per_frame = frame(:,:,i);
%         per_frame(ballground>0) = 255;
%         frame(:,:,i)= per_frame;
%     end
%     imshow(frame)
%     [x,y] = calcenter(ballground);
%     count = count+1;
%     hold on
%     lines(count,:) = [x,y];
%     plot(lines(:,1),lines(:,2),'r-');
%     [x_min,x_max,y_min,y_max] = calbbox(ballground);
%     draw_lines(x_min,x_max,y_min,y_max)
%     hold on
%     title('��ɫΪ�켣/��ɫΪ����ָ���/��ɫΪ�������')
%     function draw_lines(x_min,x_max,y_min,y_max)
%         hold on
%         liness = [x_min,x_min,x_max,x_max,x_min;y_min,y_max,y_max,y_min,y_min];
%         plot(liness(1,:),liness(2,:))
%         hold off;
%     end
%   function [x_min,x_max,y_min,y_max] = calbbox(I)
%         [rows,cols] = size(I); 
%         x = ones(rows,1)*[1:cols];
%         y = [1:rows]'*ones(1,cols);   
%         rows = I.*x;
%         x_max = max(rows(:))+2;
%         rows(rows==0) = x_max;
%         x_min = min(rows(:))-2;
%         rows = I.*y;
%         y_max = max(rows(:))+2;
%         rows(rows==0) = y_max;
%         y_min = min(rows(:))-2;
%   end
%%

    function [x,y] = initial()
%         ����Ҫ��������������ĳ�ʼ��
%         ʾ������
         [x,y] = ginput(1);
         x = int16(x);
         y = int16(y);
    end

    function ballground = segment(frame,firstframe,x,y)
%         ����Ҫ���������ÿһ֡������ǰ���ָ�
%         ʾ������
        h = 25;
        w = 25;
        bw = rgb2gray(firstframe);
        bw2 = rgb2gray(frame);
        mask = zeros(size(bw2));
        mask(y-h:y+h,x-w:x+w) = 1;
        ballground = mask&(abs(bw2-bw)>20);
        B = ones(2);
        ballground = imerode(ballground,B);
        ballground = imdilate(ballground,B);
    end

    function [meanx,meany, area] = calcenter(I)
%         ����Ҫ����������������ĵ�ļ��㣬����ǰ��ͼ
%         ʾ������
        [rows,cols] = size(I); 
        mx = ones(rows,1)*[1:cols];
        my = [1:rows]'*ones(1,cols);   
        area = sum(sum(I)); 
        meanx = int16(sum(sum(I.*mx))/area); 
        meany = int16(sum(sum(I.*my))/area);
    end

    function [distance] = caldistance(x1,y1,x2,y2)
        distance = ((x2 - x1)^2 + (y2 - y1)^2) ^(0.5);
    end
  
end