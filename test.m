clear all;
close all;
fileLoc = '/home/sun/WorkSpace/Ros_workspace/catkin_lidar_camera_calibration/src/lidar_camera_calibration/conf/points.txt';
file = fopen(fileLoc, 'r');
numPoints = str2double(fgetl(file));
tline = fgetl(file);
index = 2;
lidarPoints = [];
while(ischar(tline) && index < 10)
    linesplits = strsplit(tline);
    lidarPoints = [lidarPoints; str2double(linesplits)];
    index = index + 1;
    tline = fgetl(file);
end

cameraPoints= [];

while(ischar(tline) && index < 18)
    linesplits = strsplit(tline);
    cameraPoints = [cameraPoints; str2double(linesplits)];
    index = index + 1;
    tline = fgetl(file);
end

fclose(file);

% Estimated transformation
T = [  0.999772 -0.0057305  -0.020559  0.0529713;
 0.0076423   0.995528  0.0941526 -0.0373865;
 0.0199275 -0.0942882   0.995345  -0.491598;
         0          0          0          1];
 
figure
xlabel('X');
ylabel('Y');
zlabel('Z');

homolidarPoints = lidarPoints';
homolidarPoints = [homolidarPoints; ones(1,size(lidarPoints,1))];

transformedhomolidarPoints = T * homolidarPoints;

hold on;
scatter3(cameraPoints(:,1),cameraPoints(:,2),cameraPoints(:,3),'filled','r');
scatter3(transformedhomolidarPoints(1,:),transformedhomolidarPoints(2,:),transformedhomolidarPoints(3,:),'filled','b');
legend('camera points', 'lidar points');

thlidarPointsTranspose = transformedhomolidarPoints(1:3,:)';
postError = (cameraPoints(:) - thlidarPointsTranspose(:)).^2;
postRmse = sqrt(sum(postError(:))/8);

disp(postRmse);
