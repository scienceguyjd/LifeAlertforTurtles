% main file for reading in data from the Arduino and updating our tracking
% and our predictions
% 6 represents numbers of dimensions we are calculating
% TODO make 6 not a hard coded number
fname = 'turtleData.csv';
fid = fopen(fname);
Z = [0;0;0;0;0;0]; % initial observation
% How many samples into the future we want to predict
% 10 minutes * 60 sec/min * 25 samples/sec
%N = 10 * 60 * 25;
N = 50;
% tracker for length of Z
i = 1;
% counter for drawing once every second
counter = 0;
tline = fgets(fid);
   for i = 1:5
     accel = strsplit(tline, ',');
     accel = cell2mat(accel);
     Z(:, i) = [ 0; 0; 0; 0; accel(1); accel(2)];  
   end
   
while 1
   tline = fgets(fid);
   % get an initial amount of data
   while ischar(tline)
     accel = strsplit(tline, ',');
     accel = cell2mat(accel);
     counter = counter +1;
     Z(:, i) = [ 0; 0; 0; 0; accel(1); accel(2)];
     
     Xpredict = kalmanPredictionOld(Z, N);
     if counter == 25
         % draw graph of past positions
         plot(Xpredict(1,:), Xpredict(2,:));
         title('Position of turtle');
         counter = 0;
         pause(0.3);
     end 
     i = i + 1;
   end 
end