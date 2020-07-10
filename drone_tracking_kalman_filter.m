v = VideoReader('Video_37.avi');
N_FRAME = 200;
W = v.Width;
H = v.Height;
fid = fopen('Video_37.txt');

traj = nan(500,4);
i = 1;
while ~feof(fid)
    line = fgetl(fid);
    
%     disp(line)
%     pause
    items = split(line,' ');
    disp(items)
    if length(items)>= 5
        items = split(items{4},',');
        x = str2num(items{1}(2:end));
        y = str2num(items{2}(1:end));
        x2 = str2num(items{3}(1:end));
        y2 = str2num(items{4}(1:end-1));
        traj(i,1:4) = [y,x,y2-y,x2-x]; %[y x h w]
    end
    i = i+1;
    
end


%%
figure(1)

% m,n,A
x = nan(4,1);
vel = [0,0];
xcov = eye(4,4)*1;
% x_next = nan(4,1);
xcov_next = eye(4,4)*1;
y = nan(2,1);
y_next = nan(2,1);
fs = v.FrameRate;
dt = 1/fs;
A = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
B = [1 0 0 0;0 0 1 0];

Sn = [1, 0; 0, 1]*1;
Sv = [dt^2/2, 0; 0, dt];
Sv = [Sv, zeros(2,2); zeros(2,2), Sv];



%%
y = [traj(1,1) ;traj(1,2)];
v = VideoReader('Video_37.avi');

figure(1)
wh  = [10, 10];
for i_frame = 1:N_FRAME
    frame = readFrame(v);
    hold off;
    imshow(frame)
    hold on;
       
    trj_cnt = sum(isnan(traj(i_frame,:))); % # of NaN 

    if exist('y_hat','var') && ~isnan(y_hat(1)) && ~isnan(y_hat(2))
        tmp = traj(i_frame,3:4);
        if sum(isnan(tmp))==0
            wh = traj(i_frame,3:4);
        end
        rectangle('Position', [y_hat(1),y_hat(2),wh(1),wh(2)],'EdgeColor','g', 'LineWidth', 2)      
    end
    
    if trj_cnt==0 % when the drone is detected
        traj(i_frame,1:4);
        rectangle('Position', traj(i_frame,1:4),'EdgeColor','r', 'LineWidth', 3) %[y x h w]

    end
    
    pause(1/fs)
    % pause % To check frame-by-frame

    % detected position
    y_next = [traj(i_frame,1);traj(i_frame,2)];
    
    if sum(isnan(y_next)) == 0 && sum(isnan(y)) == 2 % only y is missing
%         vel = (y_next-y) * fs;
        x = [y_next(1), vel(1), y_next(2), vel(2)]';
        x_next = [y_next(1), vel(1), y_next(2), vel(2)]';
    end
    
    % kalman filter
    % Prediction
    x_next = A * x ;
    xcov_next = Sv + A * xcov * A';
    
    y_hat = B * x_next
    [y, y_next]
%     pause
    
    % Filtering
    if sum(isnan(y_next))==0 && sum(isnan(y))==0 % y & y_next are not missing
        S1 = B' * inv(Sn) * B ;
        xcov = inv(S1 + inv(xcov_next));  
        x = xcov * (S1 * pinv(B) * y + inv(xcov_next) * x_next);
    else % Only y_next missing or both y&y_next missing
        x = x_next;
        xcov = xcov_next;
    end

    
    y = y_next;
    
    
    
end


