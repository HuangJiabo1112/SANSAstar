%地图参数设置
map_row = 30; % 地图的尺寸
map_col = 30;

% 使用Bezier曲线平滑路径的地图
obslist = [1,5;1,6;1,7;1,8;2,5;2,6;2,7;2,8;3,7;4,7;5,7;
           3,1;3,2;3,3;4,1;4,2;4,3;
           7,4;8,4;8,3;9,3;10,3;11,3;12,3;13,3;14,3;
           12,13;12,14;12,15;12,16;13,16;14,16;15,16;16,16;15,17;16,17;13,13;14,13;15,13;15,12;15,11;15,10;15,9;15,8;15,7;
           8,7;9,7;10,7;8,8;9,8;8,9;9,9;
           5,10;5,11;5,12;4,12;6,12;7,12;8,12;8,13;8,14;9,13;5,14;6,14;7,14;5,15;
           1,27;1,28;1,29;1,30;2,26;2,27;2,28;2,29;2,30;3,26;3,27;3,28;3,29;4,24;4,25;4,26;4,27;4,28;4,29;5,25;5,26;5,27;5,28;5,29;6,27;7,27;6,25;7,25;8,25;
           2,23;2,22;2,21;1,22;1,21;1,20;
           2,18;3,18;3,17;3,16;3,15;3,14;
           1,12;2,12;2,11;
           10,23;11,23;12,23;13,23;12,24;13,24;13,22;13,21;13,20;13,19;9,22;8,21;9,23;10,22;8,22;8,20;9,21;7,21;14,19;15,19;16,19;
           6,18;7,18;
           12,29;13,29;14,29;15,29;13,28;12,30;13,30;15,30;
           16,25;16,26;16,27;17,27;18,27;17,25;18,25;19,25;17,24;
           18,21;19,21;20,21;21,21;19,22;19,23;19,20;19,19;
           24,1;25,1;26,1;26,2;27,2;
           28,4;28,5;28,6;29,4;29,5;29,6;30,4;
           21,2;22,2;22,3;22,4;22,5;22,6;22,7;22,8;21,5;21,6;23,5;23,6;23,7;23,8;24,5;24,7;
           19,5;
           15,5;16,5;16,4;
           18,14;19,14;20,14;21,14;19,15;20,13;19,14;21,14;22,14;19,13;19,12;20,12;21,12;22,12;19,11;19,10;18,10;21,11;21,10;
           22,26;23,26;24,26;25,26;22,27;23,27;24,27;25,27;22,28;23,28;24,28;25,28;22,29;23,29;24,29;25,29;
           21,17;21,18;
           25,12;25,13;25,14;25,15;25,16;25,17;25,18;25,19;25,20;
           28,8;29,8;30,8;28,9;
           28,14;29,14;30,14;28,13;
           27,23;28,22;29,21;29,23;29,22;28,23;
    ];

% 起点和终点
start = [2,2]; % 起点
goal = [29,29]; % 终点



% 创建地图
map = createMap(obslist,start,goal,map_row,map_col);

% 起始计时
tic

% 运行 A* 算法，获取路径
[parent,close_set,f_score] = AStar(map, start, goal, map_row, map_col);

% % 将close_set也添加到map中，方便可视化
% map = update_map(map,close_set,start);
% 画出障碍物、起点、终点以及访问过节点可视化图
draw(map,map_row, map_col);
path = reconstruct_path(goal,parent);
% 画出原A*算法的路径
drawPath(path);
% 计算路径长度
len = path_length(path);
disp('A* :');
disp(['         path length：', num2str(len)]);
[min_angle, max_angle, numAngleLargerThan45,angles] = getAngle(path);
disp(['         Number of corners with angle >= 45：', num2str(numAngleLargerThan45)]);


% 画出SANSA*的路径
path1 =[2,2;2,3;3,4;4,5;5,6;6,7;7,9;8,10;9,11;10,13;11,15;11,16;12,17;14,18;15,18;16,18;17,18;18,18;19,18;20,19;21,20;22,21;23,22;24,23;25,24;26,25;27,26;28,27;29,29;];
drawPath1(path1);
len1 = path_length(path1);
disp('SANSA* ：');
disp(['         path length：', num2str(len1)]);
[min_angle, max_angle, numAngleLargerThan45,angles] = getAngle(path1);
disp(['         Number of corners with angle >= 45：', num2str(numAngleLargerThan45)]);

pathsmoth = [2,2;2,2.5;];
len3 = 0;
len3 = len3 + path_length(pathsmoth);
drawPath3(pathsmoth);
path3 = [2,2.5;2,3;2.5,3.5];
pathsmoth = smothByBezier(path3);
len3 = len3 + path_length(pathsmoth);
drawPath3(pathsmoth);
pathsmoth = [pathsmoth(end,:);3,4;4,5;5,6;6,7;7,9;8,10;9,11;10,13;11,15;11,15.5;];
len3 = len3 + path_length(pathsmoth);
drawPath3(pathsmoth);
path3 = [11,15.5;11,16;11.5,16.5;];
pathsmoth = smothByBezier(path3);
len3 = len3 + path_length(pathsmoth);
drawPath3(pathsmoth);
pathsmoth = [pathsmoth(end,:);12,17;14,18;15,18;16,18;17,18;18,18;18.5,18;];
len3 = len3 + path_length(pathsmoth);
drawPath3(pathsmoth);
path3 = [18.5,18;19,18;19.5,18.5;];
pathsmoth = smothByBezier(path3);
len3 = len3 + path_length(pathsmoth);
drawPath3(pathsmoth);
pathsmoth = [pathsmoth(end,:);20,19;21,20;22,21;23,22;24,23;25,24;26,25;27,26;28,27;29,29;];
len3 = len3 + path_length(pathsmoth);
disp('Smoothing by Bezier: ');
drawPath3(pathsmoth);
disp(['         path length：', num2str(len3)]);

function map = createMap(obslist,start,goal,map_row,map_col)
    % 创建一个空的栅格地图，所有单元格初始化为0（白色，可通行）
    grid_map = zeros(map_row, map_col);
    
    % 指定固定的障碍物的栅格坐标  obs为为传入的障碍物位置
    obstacle_cells = obslist;
    
    % 将障碍物单元格标记为1（黑色，障碍物）
    for i = 1:size(obstacle_cells, 1)
        row = obstacle_cells(i, 2);
        col = obstacle_cells(i, 1);
        grid_map(row, col) = 1;
    end
    
    % 添加起点和终点  
    start_point = start;  % 起点坐标
    end_point = goal;  % 终点坐标
    grid_map(start_point(2), start_point(1)) = 2;  % 起点标记为2（绿色）
    grid_map(end_point(2), end_point(1)) = 3;  % 终点标记为3（蓝色）
    
    % 需要返回的地图
    map=grid_map;
end


function map = update_map(map,close_set,start)
    for i = 1:size(close_set, 1)
        % 获取当前已访问的节点坐标
        visited_node = close_set(i, :);
        if visited_node == start
            continue;
        end
        % 获取当前节点的位置
        x = visited_node(2);
        y = visited_node(1);
        map(x, y) = 4;  % 4 表示已访问
    end
end

% 画栅格地图函数drawGridMap
function draw(map,map_row, map_col)
    % 创建栅格地图图像 flipud(grid_map)
    figure;
    imshow(map, 'InitialMagnification', 'fit');
    colormap([1 1 1; 0 0 0; 1 0 0; 0 0 1; 0.8 0.8 0.8]);  
    % 使用 caxis 设置颜色映射范围
    caxis([0, 4]);  
    axis on;
    
    % 反转 y 轴以左下角为原点
    axis xy;
    
    % 添加栅格线
    for x = 1:map_row-1
        line([x+0.5, x+0.5], [0.5, map_row+0.5], 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1);
    end
    for y = 1:map_col-1
        line([0.5, map_col+0.5], [y+0.5, y+0.5], 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1);
    end
end


function drawPath(path)
    % 显示路径
    hold on;
    
    % 检查路径是否有效
    if ~isempty(path) && size(path, 1) > 1
        % 绘制路径线
        plot(path(:, 1), path(:, 2), 'r', 'LineWidth', 2);
    end

    hold off;
end

function drawPath1(path)
    % 显示路径
    hold on;
    
    % 检查路径是否有效
    if ~isempty(path) && size(path, 1) > 1
        % 绘制路径线
        plot(path(:, 1), path(:, 2), 'g-.', 'LineWidth', 2);
    end

    hold off;
end

function drawPath2(path)
    % 显示路径
    hold on;
    
    % 检查路径是否有效
    if ~isempty(path) && size(path, 1) > 1
        % 绘制路径线
        plot(path(:, 1), path(:, 2), 'm', 'LineWidth', 2);
    end

    hold off;
end

function drawPath3(path)
    % 显示路径
    hold on;
    
    % 检查路径是否有效
    if ~isempty(path) && size(path, 1) > 1
        % 绘制路径线
        plot(path(:, 1), path(:, 2), 'b', 'LineWidth', 1);
    end

    hold off;
end

function drawPath4(path)
    % 显示路径
    hold on;
    
    % 检查路径是否有效
    if ~isempty(path) && size(path, 1) > 1
        % 绘制路径线
        plot(path(:, 1), path(:, 2), 'c', 'LineWidth', 1);
    end

    hold off;
end


% A*算法的实现
function [parent,close_set,f_score] = AStar(map, start, goal,map_row, map_col)
    % 初始化OpenSet和CloseSet
    open_set = [];
    close_set = [];

    % 创建一个空的 Map 容器,用来将两个坐标存在键值对中  子节点是键，父节点是值
    parent = containers.Map(); 

    % 将起始节点加入OpenSet
    open_set = [open_set; start];

    % 起点的父节点是本身
    start_str = mat2str(start);
    parent(start_str) = start;

    % 初始化代价函数和估价函数值
    g_score = zeros(size(map));
    f_score = inf(size(map));
    
    % 起始节点的代价值设为0，估价函数值是启发式函数值
    g_score(start(2), start(1)) = 0;
    f_score(start(2), start(1)) = heuristic(start, goal);
    % f_score(start(2), start(1)) = 1.5*heuristic(start, goal);
    
    while ~isempty(open_set)
        % 计算估价函数值
        f_values = f_score(sub2ind(size(f_score), open_set(:, 2), open_set(:, 1)));
        
        % 选择OpenSet中估价函数值最小的节点
        [~, index] = min(f_values);
        current_node = open_set(index, :);
        
        % 如果当前节点是目标节点，返回路径
        if isequal(current_node, goal)
            %path = reconstruct_path(current_node,parent);
            return;
        end
        
        % 从OpenSet中移除当前节点
        open_set(index, :) = [];
        
        % 将当前节点加入CloseSet
        close_set = [close_set; current_node];
        
        % 获取当前节点的邻居节点
        neighbors = neighbor_nodes(current_node, map_row, map_col);
        
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            
            % 如果邻居节点在CloseSet中或是障碍物，跳过
            if ismember(neighbor, close_set, 'rows') || map(neighbor(2), neighbor(1)) == 1
                continue;
            end
            
            % 计算邻居节点的新g_score
            tentative_g_score = g_score(current_node(2), current_node(1)) + update_g(current_node,neighbor);
            
            % 如果邻居节点不在OpenSet中，或新g_score更小，更新邻居节点信息
            if ~ismember(neighbor, open_set, 'rows') || tentative_g_score < g_score(neighbor(2), neighbor(1))
                g_score(neighbor(2), neighbor(1)) = tentative_g_score;
                f_score(neighbor(2), neighbor(1)) = tentative_g_score + heuristic(neighbor, goal);
                neighbor_str = mat2str(neighbor);
                parent(neighbor_str) = current_node; % 将邻居节点的父节点是当前以current_node为中心搜索的点
                open_set = [open_set; neighbor];
            end
        end
    end

    % 如果OpenSet为空，且没有找到路径，返回空路径
    path = [];
end


% g 是指父节点到邻节点之间的代价
function g = update_g(parent,child) 
    g = sqrt((parent(1) - child(1))^2 + (parent(2) - child(2))^2);
end


% 启发函数h的实现
function h = heuristic(a, b)
    % 启发式函数，这里使用欧式距离
    h = sqrt((a(1) - b(1))^2 + (a(2) - b(2))^2);
end

% 计算两点之间的欧式距离
function eu = cal_eu(a,b)
    eu = sqrt((a(1) - b(1))^2 + (a(2) - b(2))^2);
end

% 计算当前节点到以A,B,C为参数的直线的欧式距离
function d = calculateDistance(node,A,B,C)
    d = abs(A * node(1) + B * node(2) + C) / sqrt(A^2 + B^2);
end

% 计算直线的参数A,B,C
function [A,B,C] = cal_ABC(start,goal)
    A = goal(2) - start(2);
    B = start(1) - goal(1);
    C = goal(1)*start(2) - goal(2)*start(1);
end


% 获取邻居的函数实现
function neighbors = neighbor_nodes(node, map_row, map_col)
    % 获取8个方向的邻居节点
    neighbors = [node(1)-1, node(2);  % 左
                 node(1)+1, node(2);  % 右
                 node(1), node(2)-1;  % 上
                 node(1), node(2)+1;  % 下
                 node(1)-1, node(2)-1; % 左上
                 node(1)+1, node(2)-1; % 右上
                 node(1)-1, node(2)+1; % 左下
                 node(1)+1, node(2)+1; % 右下
                ];
    
    % 剔除超出地图边界的邻居
    neighbors = neighbors(neighbors(:, 1) >= 1 & neighbors(:, 1) <= map_row & ...
                          neighbors(:, 2) >= 1 & neighbors(:, 2) <= map_col, :);
end


% 获取路径的函数实现
function path = reconstruct_path(current,parent)
    % 从目标节点回溯路径
    path = current;
    while ~isequal(parent(mat2str(current)),current)
        current_str = mat2str(current); % 将current转为字符串
        current = parent(current_str); % 根据current的坐标找到 current的父节点，得到的父节点是矩阵类型
        path = [current; path];
    end
end

% 获取路径长度
function length = path_length(path)
    % 初始化路径长度为0
    length = 0;
    segment_start = path(1,:);
    for i=2:size(path,1)
        segment_end = path(i, :);
        % 计算每段路径的长度
        length = length + cal_eu(segment_start,segment_end);
        segment_start = path(i, :);
    end
end

function path = smothByBezier(path)
    t = linspace(0,1);
    t = t';
    path = (1-t).^2*path(1,:) + 2*(1-t).*t*path(2,:) + t.^2*path(3,:);
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
        if angle <= 1
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

