function [Xpredict] = kalmanPredictionOld(Z, N)
    % Z is the current, and past observations, with newest observation at
    % Z(end) with [x y dx/dt dy/dt]'
    % N is the number of frames to predict
     f_s = 25;
    % X is a state variable (position and speed) X = [x y dx/dt dy/dt]';
    % A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
%     A = [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1];
%     H = [1 0 0 0; 0 1 0 0; 0 0 0 0; 0 0 0 0];
    A = [1 0 1/f_s 0 1/(f_s)^2 0; 0 1 0 1/f_s 0 1/(f_s)^2; 0 0 1 0 1/f_s 0; ...
        0 0 0 1 0 1/f_s; 0 0 0 0 1 0; 0 0 0 0 0 1];
    H = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0;...
    0 0 0 0 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
    % Noise for the position measurement
    R = 0.5*eye(6);
    % Preallocate
    Xpredict = zeros(6,N);
    P = zeros(6,6,N);
    Ppredict = zeros(6,6,N);
    K = zeros(6,6,N);

    % initialize
    Xstate(:,1) = [0;0;0;0;0;0];
    P(:,:,1) = eye(6);

    for j=2:length(Z)-1
     j
     % Time Update
     Xpredict(:,j) = A*Xstate(:,j-1);
     Ppredict(:,:,j) = A*P(:,:,j-1)*A';
     % Kalman Correction
     K(:,:,j) = Ppredict(:,:,j)*H'*inv(H*Ppredict(:,:,j)*H'+R);
     Xstate(:,j) = Xpredict(:,j) + K(:,:,j)*(Z(:,j)-H*Xpredict(:,j));
     P(:,:,j) = (eye(6)-K(:,:,j)*H)*Ppredict(:,:,j);
    end

    for j = length(Z):N
     % Time Update
     Xpredict(:,j) = A*Xstate(:,j-1);
     Ppredict(:,:,j) = A*P(:,:,j-1)*A';
     % Kalman Correction
     K(:,:,j) = Ppredict(:,:,j)*H'*inv(H*Ppredict(:,:,j)*H'+R);
     Xstate(:,j) = Xpredict(:,j);
     P(:,:,j) = (eye(6)-K(:,:,j)*H)*Ppredict(:,:,j);
    end
    
    Xpredict = Xpredict(:,2:end);
    
    %plots for debugging
%     plot(Xpredict(1,:),'b');
%     figure
%     plot(Xpredict(2,:),'r');
%     figure
%     plot(Xpredict(3,:),'g');
%     figure
%     plot(Xpredict(4,:),'k');

end