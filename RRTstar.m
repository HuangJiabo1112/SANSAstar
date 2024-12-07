clear all; close all; clc;
%% 参数初始化
x_I = 2; y_I = 2;           % 设置初始点
x_G = 99; y_G = 99;       % 设置目标点
GoalThreshold = 5;         % 设置目标点阈值
Delta = 5;                 % 设置扩展步长 default:30
RadiusForNeib = 5;          % RRT* 新的参数：rewire 的范围,半径 r
MaxIterations = 3000;       % 最大迭代次数
UpdateTime = 50;            % 更新路径的时间间隔
DelayTime = 0.0;            
%% 建树初始化:T 是树,v 是节点
T.v(1).x = x_I;             % 把起始节点加入到T中
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         % 节点的父节点坐标:起点的父节点是其本身
T.v(1).yPrev = y_I;
T.v(1).totalCost = 0;       % 从起始节点开始的累计 cost，这里取欧氏距离
T.v(1).indPrev = 0;         % 父节点的索引
%% 开始构建树
figure(1);

%% 建图
% % 读取30x30读图
% data = load('map30x30.mat');
% Imp1 = data.map1;
% Imp1(2,2) = 0;
% Imp1(29,29) = 0;

% % 读取60x60读图
% data = load('map60x60.mat');
% Imp1 = data.map1;
% Imp1(2,2) = 0;
% Imp1(59,59) = 0;
% 
% 读取100x100读图
data = load('map100x100.mat');
Imp1 = data.map1;
Imp1(2,2) = 0;
Imp1(99,99) = 0;


Imp = ~Imp1;
imshow(Imp,'InitialMagnification', 'fit')
axis on;
axis xy;
xL = size(Imp,1);   % 地图 x 轴长度
yL = size(Imp,2);   % 地图 y 轴长度
hold on
% plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点和目标点
% plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
rectangle('Position', [x_I-0.5, y_I-0.5, 1, 1], 'EdgeColor', 'r', 'LineWidth', 1, 'FaceColor','r'); 
rectangle('Position', [x_G-0.5, y_G-0.5, 1, 1], 'EdgeColor', 'b', 'LineWidth', 1, 'FaceColor','b');
count = 1;
% 保存重新选择的父节点到新采样节点的绘图句柄
pHandleList = [];
% 保存新采样点的绘图句柄
lHandleList = [];
% 保存最优路劲的绘图句柄
resHandleList = [];
% 是否找到路径
findPath = 0;
update_count = 0;
% 保存最优路径点
path.pos = [];
drawTime = 0;
RRTstarTime = 0;

% RRT* 达到迭代次数才结束算法
for iter = 1 : MaxIterations
    tic;
    % Step 1: 在地图中随机采样一个点 x_rand (Sample)
    x_rand = [unifrnd(0, xL),unifrnd(0, yL)];
    
    % Step 2: 遍历树，从树中找到最近邻近点 x_near (Near)
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;
    
    % T.v 按行向量存储，size(T.v, 2)获得节点总数
    for i = 2 : size(T.v,2)	
    	distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);   % 两节点间距离
        if(distance < minDis)
            minDis = distance;
            minIndex = i;
        end
    end
    
    % 找到了当前树中离 x_rand 最近的节点
    x_near(1) = T.v(minIndex).x;    
    x_near(2) = T.v(minIndex).y;
    temp_parent = minIndex;                        % 临时父节点 x_near 的索引
    temp_cost = T.v(minIndex).totalCost + Delta;   % 临时累计代价 x_i ->(totalCost) x_near ->(Delta) x_rand

    % Step 3: 扩展得到 x_new 节点 (Steer)
    theta = atan2((x_rand(2) - x_near(2)), (x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;  
    %plot(x_rand(1), x_rand(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
    %plot(x_new(1), x_new(2), 'bo', 'MarkerSize',10, 'MarkerFaceColor','b');
    
    % 检查节点是否是 collision-free
    if ~collisionChecking(x_near, x_new, Imp) 
        continue;   % 有障碍物
    end

    % Step 4: 在以 x_new 为圆心,半径为 R 的圆内搜索节点 (NearC)
    % 每次循环要把队列清空
    disToNewList = [];      % 保存邻居点到 x_new 的欧式距离（cost） 
    nearIndexList = [];     % 保存邻居点的索引
    % count 是采样点的数量，这里遍历所有的采样点
    for index_near = 1 : count
        disTonew = sqrt((x_new(1) - T.v(index_near).x)^2 + (x_new(2) - T.v(index_near).y)^2);
        if(disTonew < RadiusForNeib)                        % 满足条件: 欧氏距离小于 R
            disToNewList = [disToNewList disTonew];         % 满足条件的所有节点到 x_new 的欧式距离 cost
            nearIndexList = [nearIndexList index_near];     % 满足条件的所有邻居节点基于树 T 的索引
        end
    end
    
    % Step 5: 重新选择 x_new 的父节点，使 x_new 的累计 cost 最小 (ChooseParent)
    % cost_index 是邻居节点的索引
    for cost_index = 1 : length(nearIndexList)
        % 计算每个邻居节点作为 x_new 父节点的总代价：x_i -> x_near -> x_new
        costToNew = T.v(nearIndexList(cost_index)).totalCost + disToNewList(cost_index);
        % temp_cost 为刚生成 x_new 时的初始代价
        if(costToNew < temp_cost)
            % 邻居节点到 x_new 的总成本更小就更新为 x_new 的父节点
            x_mincost(1) = T.v(nearIndexList(cost_index)).x;
            x_mincost(2) = T.v(nearIndexList(cost_index)).y;
            
            % 检测父子节点的连线是否会碰到障碍物
            if ~collisionChecking(x_mincost, x_new, Imp) 
            	continue;   % 有障碍物
            end
            
        	temp_cost = costToNew;                      % 保存最小成本
        	temp_parent = nearIndexList(cost_index);    % 保存最小成本的邻居节点
        end
    end
    
    % Step 6: 将 x_new 插入树 T (AddNodeEdge)，count 也是最新节点的索引
    count = count + 1;
    
    T.v(count).x = x_new(1);          
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = T.v(temp_parent).x;      % 最新节点的父节点为重新选择的邻居父节点 temp_parent
    T.v(count).yPrev = T.v(temp_parent).y;      
    T.v(count).totalCost = temp_cost;           % 最新节点的总成本为重新选择的邻居父节点的最下成本 costToNew
    T.v(count).indPrev = temp_parent;           % 重新选择的父节点 x_near 的 index
    
    RRTstartime = toc;
    RRTstarTime = RRTstarTime + RRTstartime;

    % 在重新选择的父节点到 x_new 之间画一条蓝色的线
    % T.v(count).xPrev 为第一步重新选择的父节点
    l_handle = plot([T.v(count).xPrev, x_new(1)], [T.v(count).yPrev, x_new(2)], 'b', 'Linewidth', 2);
    % 在新采样的 x_new 处画个黑色 k 圆圈 o 标记 Marker
    p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
    
    % 绘图的句柄索引即为 count
    pHandleList = [pHandleList p_handle];    
    lHandleList = [lHandleList l_handle];
    pause(DelayTime);
    
    % Step 7: 对邻居节点重新布线 (rewire)，计算是否需要把 x_new 换成邻居节点的父节点
    for rewire_index = 1 : length(nearIndexList)
        % temp_parent 为 x_new 的最有父节点，不用重新布线
        if(nearIndexList(rewire_index) ~= temp_parent)
            % 计算起点 -> x_new -> 邻居节点的总代价
            newCost = temp_cost + disToNewList(rewire_index);
            % 把 x_new 作为邻居节点的总成本 newCost 小于原始邻居节点的总成本，则替换邻居节点的父节点为 x_new
            if(newCost < T.v(nearIndexList(rewire_index)).totalCost)
                % 得到可以优化的邻居节点坐标
                x_neib(1) = T.v(nearIndexList(rewire_index)).x;
                x_neib(2) = T.v(nearIndexList(rewire_index)).y;
                
                % 判断父子节点连线是否会碰到障碍物
                if ~collisionChecking(x_neib, x_new, Imp) 
                    continue;   % 有障碍物
                end
                
                T.v(nearIndexList(rewire_index)).xPrev = x_new(1);      % 把邻居节点的父节点替换为 x_new
                T.v(nearIndexList(rewire_index)).yPrev = x_new(2);
                T.v(nearIndexList(rewire_index)).totalCost = newCost;   % 把邻居节点的总成本也替换为新的
                T.v(nearIndexList(rewire_index)).indPrev = count;       % 设置优化的邻居节点的父节点索引为 x_new 的索引

                % 在 x_new 和优化父节点为 x_new 的邻居节点之间绘制红色的线段
                lHandleList(nearIndexList(rewire_index)) = plot([T.v(nearIndexList(rewire_index)).x, x_new(1)], [T.v(nearIndexList(rewire_index)).y, x_new(2)], 'b', 'Linewidth', 2);
            end
        end
    end
    
    tic;
    % Step 8: 检查是否到达目标点附近 
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < GoalThreshold && ~findPath)    % 找到目标点，此条件只进入一次
        % 设置已经找到路径，后续利用这个标记把优化的轨迹画出来
        findPath = 1;

        count = count + 1;    % 手动将 Goal 加入到树中
        Goal_index = count;
        T.v(count).x = x_G;          
        T.v(count).y = y_G; 
        T.v(count).xPrev = x_new(1);     % 目标节点的父节点为最新的 x_new
        T.v(count).yPrev = x_new(2);
        T.v(count).totalCost = T.v(count - 1).totalCost + disToGoal; % 目标节点的总成本为父节点成本 + 到终点的成本
        T.v(count).indPrev = count - 1;     % 其父节点 x_near 的 index
    end
    RRTstartime = toc;
    RRTstarTime = RRTstarTime + RRTstartime;
    
    % 找到路径就回溯，但是不结束 RRT* 算法
    if(findPath == 1)
        update_count = update_count + 1;
        % UpdateTime = 50 次更新一次路径
        if(update_count == UpdateTime)
            update_count = 0;
            j = 2;
            % path 的第一个元素为路径的终点
            path.pos(1).x = x_G; 
            path.pos(1).y = y_G;
            pathIndex = T.v(Goal_index).indPrev;
            
            while 1
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;    % 沿终点回溯到起点
                if pathIndex == 0
                    break
                end
                j = j + 1;
            end
            
            % 每次绘制 RRT* 优化的路径前，把上次优化的路径句柄清空
            for delete_index = 1:length(resHandleList)
            	delete(resHandleList(delete_index));
            end
            
            % 画出每次迭代优化的绿色 RRT* 路径（从终点向起点绘制路径）
            % path 的第一个元素为路径的终点，最后一个元素为路径的起点
            for j = 2 : length(path.pos)
                res_handle = plot([path.pos(j).x; path.pos(j - 1).x;], [path.pos(j).y; path.pos(j - 1).y], 'g', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
        end
    end
	pause(DelayTime); % 暂停 DelayTime s,使得 RRT* 扩展过程容易观察
    RRTstarTime = RRTstarTime + RRTstartime;
end


% 再次清空路径，为下一步重新绘制最终优化的路径做准备
for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end

% 重新绘制最优路径 path，保证不会被其他颜色的路径遮挡
for j = 2 : length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j - 1).x;], [path.pos(j).y; path.pos(j - 1).y], 'r', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end

disp(['Running time：', num2str(RRTstarTime), ' s']);
start = [x_I,y_I];
Path = [];
for i = 1 : length(path.pos)
	node = [path.pos(i).x, path.pos(i).y];
    Path = [node; Path];
end
len = path_length(Path,start);
disp(['distance：', num2str(len)]);
[min_angle, max_angle, numAngleLargerThan45,angles] = getAngle(Path);
disp(['Min corner angle：', num2str(min_angle)]);
disp(['Max corner angle：', num2str(max_angle)]);
disp(['Number of corners with angle >= 45：', num2str(numAngleLargerThan45)]);
            
% 计算两点之间的欧式距离
function eu = cal_eu(a,b)
    eu = sqrt((a(1) - b(1))^2 + (a(2) - b(2))^2);
end

% 获取路径长度
function length = path_length(path,start)
    % 初始化路径长度为0
    length = 0;
    segment_start = start;
    for i=2:size(path,1)
        segment_end = path(i, :);
        % 计算每段路径的长度
        length = length + cal_eu(segment_start,segment_end);
        segment_start = path(i, :);
    end
end

function [min_angle, max_angle, numAngleLargerThan45,angles] = getAngle(path)
    length = size(path, 1);
    angles = zeros(length - 2, 1);
    numAngleLargerThan45 = 0;

    for i = 1:size(path, 1) - 2
        node1 = path(i, :);
        node2 = path(i + 1, :);
        node3 = path(i + 2, :);

        l1 = node2 - node1;
        l2 = node3 - node2;
        l1dotl2 = dot(l1,l2);
        norm_l1 = norm(l1); 
        norm_l2 = norm(l2);
        cos_theta = l1dotl2 / (norm_l1 * norm_l2);
        rad = acos(cos_theta);
        angle = rad2deg(rad);
        if angle <= 1 || angle >=180
            angles(i) = inf;
        else
            angles(i) = angle;
            if angle >= 45
                numAngleLargerThan45 = numAngleLargerThan45 + 1;
            end
        end
    end

    min_angle = min(angles);
    valid_angles = angles(~isinf(angles)); % 过滤掉 Inf 值
    max_angle = max(valid_angles);
end

