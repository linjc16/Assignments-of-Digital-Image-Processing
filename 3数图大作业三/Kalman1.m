function  Kalman1()
% 根据第一次播放图像，判断球的标签并修改trackid的值
trackid = 1;
% 修改播放的id值
vedioid = 3;
videoname = sprintf('./videos/%d.mp4',vedioid);
% 选取相应的半径
radius_4 = 0.19 / 2 ; %4号足球直径19cm
radius_5 = 0.215 / 2; %5号足球直径21.5cm
if(vedioid == 0)
    radius_pri = radius_4;
else
    radius_pri = radius_5;
end
% radius_pri = radius_4; % 根据实际球的大小进行修改
% 视频1-8使用参数 20 400， 视频0使用参数 200 1200
if vedioid == 0
    MinimumBlobArea = 200; 
    MaximumBlobArea = 1400;
else
    
    MinimumBlobArea = 20;
    MaximumBlobArea = 400;
end

% 记录下初始坐标并存入lines中
count = 0;
lines = [];
centrid = [];
radius = [];
lastx = [];
lasty = [];
obj = setupSystemObjects();
tracks = initializeTracks(); 
nextId = 1; 
while ~isDone(obj.reader)
    frame = readFrame();
    [centroids, bboxes, mask] = detectObjects();
    predictNewLocationsOfTracks();
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();

    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();
    displayTrackingResults();
    if ~isempty(tracks)
        if(trackid ~=1)
            len = length(tracks);
            for iter = 1:len
               if tracks(iter).id == trackid 
                   % 读取中心坐标 并将bbox段边设为球的直径长度
                   ball_x = double(tracks(trackid).bbox(1) + 0.5 * tracks(trackid).bbox(3));
                   ball_y = double(tracks(trackid).bbox(2) + 0.5 * tracks(trackid).bbox(4));
                   r = double(min(tracks(trackid).bbox(3:4)) * 0.5);
                   if (count ~= 0)
                       if((abs(ball_x - lines(count,1)) > 35 || abs(ball_y - lines(count,2)) > 35))
                           continue;
                       end
                   end
                   centrid = [centrid; ball_x, ball_y];
                   radius = [radius; r];
                   count = count + 1;
                   hold on
                   lines(count,:) = [ball_x,ball_y];
                   plot(lines(:,1),lines(:,2),'r-');
                   pause(0.001);
                   hold off
                   break;
               end
            end
        else
            % 读取中心坐标 并将bbox段边设为球的直径长度
            ball_x = double(tracks(trackid).bbox(1) + 0.5 * tracks(trackid).bbox(3));
            ball_y = double(tracks(trackid).bbox(2) + 0.5 * tracks(trackid).bbox(4));
            r = double(min(tracks(trackid).bbox(3:4)) * 0.5);
            if (count ~= 0)
                if((abs(ball_x - lines(count,1)) > 35 || abs(ball_y - lines(count,2)) > 35))
                    continue;
                end
            end
            centrid = [centrid; ball_x, ball_y];
            radius = [radius; r];
            count = count + 1;
            hold on
            lines(count,:) = [ball_x,ball_y];
            plot(lines(:,1),lines(:,2),'r-');
            pause(0.001);
            hold off
        end

    end
end
%% 计算速度
Startid = 7;
Endid = size(radius,1) - 5;
r_mean = sum(radius(Startid:Endid,1)) / (Endid - Startid);
ballshift  = [];
vball = [];
shiftcount = 0;
vballcount = 0;
for iter = Startid:Endid-1
    shiftcount = shiftcount + 1;
    ballshift(shiftcount,1) = caldistance(centrid(iter,1),centrid(iter,2),centrid(iter+1,1),centrid(iter+1,2));
end
for iter = Startid + 1:Endid - 1
    vballcount = vballcount + 1;
    shift = caldistance(centrid(iter-1,1),centrid(iter-1,2),centrid(iter+1,1),centrid(iter+1,2));
    vball(vballcount,1) = shift / 2;
end
v_mean = sum(ballshift) / shiftcount(1,1);
v_mean = v_mean / r_mean * radius_pri * 240;
vball = vball / r_mean * radius_pri * 240;
%% 计算速度(指数移动平均)
alpha  = 0.4;
v_mean_EMA = alpha * vball(1,1);
for iter = 2:vballcount - 10
    if(vball(iter,1) - v_mean_EMA > 10 || vball(iter,1) - v_mean_EMA < -10)
        continue;
    end
    v_mean_EMA = alpha * vball(iter,1) + (1 - alpha) * v_mean_EMA;
end
% 计算最大速度
v_max = max(vball(vball<30));


%% 一些函数
     function obj = setupSystemObjects()
        obj.reader = vision.VideoFileReader(videoname);
        
        obj.maskPlayer = vision.VideoPlayer('Position', [740, 200, 700, 400]);
        obj.videoPlayer = vision.VideoPlayer('Position', [20, 200, 700, 400]);

        obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
            'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);

        obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', MinimumBlobArea,'MaximumBlobArea',MaximumBlobArea);
     end
     function tracks = initializeTracks()
        tracks = struct(...
            'id', {}, ...
            'bbox', {}, ...
            'kalmanFilter', {}, ...
            'age', {}, ...
            'totalVisibleCount', {}, ...
            'consecutiveInvisibleCount', {});
     end
     function frame = readFrame()
        frame = obj.reader.step();
     end
     function [centroids, bboxes, mask] = detectObjects()

        % 检测前景
        mask = obj.detector.step(frame);

        mask = imopen(mask, strel('rectangle', [3,3]));
        mask = imclose(mask, strel('rectangle', [15, 15]));
        mask = imfill(mask, 'holes');

        [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
     end
     function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            bbox = tracks(i).bbox;

            % 使用卡尔曼滤波器进行预测
            predictedCentroid = predict(tracks(i).kalmanFilter);

            predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
            tracks(i).bbox = [predictedCentroid, bbox(3:4)];
        end
     end
     function [assignments, unassignedTracks, unassignedDetections] = detectionToTrackAssignment()

        nTracks = length(tracks);
        nDetections = size(centroids, 1);

        % Compute the cost of assigning each detection to each track.
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end

        % Solve the assignment problem.
        costOfNonAssignment = 20;
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, costOfNonAssignment);
     end
     function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);
            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);

            % Correct the estimate of the object's location
            % using the new detection.
            correct(tracks(trackIdx).kalmanFilter, centroid);

            % Replace predicted bounding box with detected
            % bounding box.
            tracks(trackIdx).bbox = bbox;

            % Update track's age.
            tracks(trackIdx).age = tracks(trackIdx).age + 1;

            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
     end
     function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;
        end
     end
     function deleteLostTracks()
        if isempty(tracks)
            return;
        end

        invisibleForTooLong = 20;
        ageThreshold = 8;

        % Compute the fraction of the track's age for which it was visible.
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;

        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

        % Delete lost tracks.
        tracks = tracks(~lostInds);
     end
     function createNewTracks()
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);

        for i = 1:size(centroids, 1)

            centroid = centroids(i,:);
            bbox = bboxes(i, :);

            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [200, 50], [100, 25], 100);

            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0);

            % Add it to the array of tracks.
            tracks(end + 1) = newTrack;

            % Increment the next id.
            nextId = nextId + 1;
        end
     end
     function displayTrackingResults()
        % Convert the frame and the mask to uint8 RGB.
        frame = im2uint8(frame);
        mask = uint8(repmat(mask, [1, 1, 3])) .* 255;

        minVisibleCount = 8;
        if ~isempty(tracks)

            % Noisy detections tend to result in short-lived tracks.
            % Only display tracks that have been visible for more than
            % a minimum number of frames.
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount;
            reliableTracks = tracks(reliableTrackInds);

            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.
            if ~isempty(reliableTracks)
                % Get bounding boxes.
                bboxes = cat(1, reliableTracks.bbox);

                % Get ids.
                ids = int32([reliableTracks(:).id]);

                % Create labels for objects indicating the ones for
                % which we display the predicted rather than the actual
                % location.
                labels = cellstr(int2str(ids'));
                predictedTrackInds = ...
                    [reliableTracks(:).consecutiveInvisibleCount] > 0;
                isPredicted = cell(size(labels));
                isPredicted(predictedTrackInds) = {' predicted'};
                labels = strcat(labels, isPredicted);

                % Draw the objects on the frame.
                frame = insertObjectAnnotation(frame, 'rectangle', ...
                    bboxes, labels);

                % Draw the objects on the mask.
                mask = insertObjectAnnotation(mask, 'rectangle', ...
                    bboxes, labels);
            end
        end

        % Display the mask and the frame.
        obj.maskPlayer.step(mask);
        obj.videoPlayer.step(frame);
     end
     function [distance] = caldistance(x1,y1,x2,y2)
        distance = ((x2 - x1)^2 + (y2 - y1)^2) ^(0.5);
     end
end

