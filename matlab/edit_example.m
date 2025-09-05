clear; close all; clc;

%% --- User-defined Threshold for WN Classification ---
wn_threshold = 0.5;   % <-- 用户可在此修改阈值

%% --- 1. Original Closed Polygon (Staircase Example) ---
vertices_closed = [
    0,0;
    4,0;
    4,1;
    3,1;
    3,2;
    5,2;
    5,4;
    0,4
];
edges_closed = [vertices_closed, circshift(vertices_closed, -1)];
edges_closed(end,:) = [vertices_closed(end,:), vertices_closed(1,:)];

%% --- 2. Define Various Open Modification Schemes ---
modifications = {
    struct('edge_idx',2,'translation',[0,-0.5],'rotation',pi/12),   
 %   struct('edge_idx',3,'translation',[0,2.5],'rotation',0),   
    struct('edge_idx',4,'translation',[-0.5,0],'rotation',0),       
    struct('edge_idx',6,'translation',[0.5,0.5],'rotation',-pi/8),  
    struct('edge_idx',1,'translation',[0,-1],'rotation',pi/6),      
    struct('edge_idx',5,'translation',[0.8,-0.3],'rotation',pi/4)   
};
num_mods = numel(modifications);
edges_modified_all = cell(num_mods,1);

%% --- 3. Generate All Open Polygons ---
for k = 1:num_mods
    mod = modifications{k};
    edges_open = edges_closed;
    original_p1 = edges_closed(mod.edge_idx, 1:2);
    original_p2 = edges_closed(mod.edge_idx, 3:4);
    center = (original_p1 + original_p2)/2;
    p1_trans = original_p1 - center;
    p2_trans = original_p2 - center;
    R = [cos(mod.rotation), -sin(mod.rotation); sin(mod.rotation), cos(mod.rotation)];
    p1_rot = (R*p1_trans')';
    p2_rot = (R*p2_trans')';
    p1_mod = p1_rot + center + mod.translation;
    p2_mod = p2_rot + center + mod.translation;
    edges_open(mod.edge_idx,:) = [p1_mod, p2_mod];
    edges_modified_all{k} = edges_open;
end

%% --- 4. Grid Point Generation (for WN calculation) ---
min_x = -2; max_x = 7;
min_y = -3; max_y = 6;
resolution = 0.05;
[X,Y] = meshgrid(min_x:resolution:max_x, min_y:resolution:max_y);
query_points = [X(:), Y(:)];

%% --- 5. Calculate WN Fields ---
wn_closed = compute_winding_number_field(query_points, edges_closed);
WN_closed_reshaped = reshape(wn_closed, size(X));
WN_modified_all = zeros(size(X,1), size(X,2), num_mods);
for k = 1:num_mods
    wn_open = compute_winding_number_field(query_points, edges_modified_all{k});
    WN_modified_all(:,:,k) = reshape(wn_open, size(X));
end

%% --- Custom Blue-White-Red Colormap ---
% You can adjust the number of steps (e.g., 64, 128, 256)
N_colors = 256; 
custom_colormap = [
    linspace(0,0,N_colors/2)', linspace(0,0,N_colors/2)', linspace(1,0,N_colors/2)';  % Blue to White
    linspace(0,1,N_colors/2)', linspace(0,0,N_colors/2)', linspace(0,0,N_colors/2)';  % White to Red
];

% More refined blue-white-red (white in the middle)
custom_colormap_bwr = [
    linspace(0,1,N_colors/2+1)', linspace(0,1,N_colors/2+1)', linspace(1,1,N_colors/2+1)'; % Blue to white
    linspace(1,1,N_colors/2)', linspace(1,0,N_colors/2)', linspace(1,0,N_colors/2)'; % White to red
];
custom_colormap_bwr = custom_colormap_bwr(1:N_colors,:); % Ensure correct size

%% --- 6. Visualization: Figure 1 - Winding Number Fields ---
total_wn_plots = 1 + num_mods;
num_cols_layout = ceil(sqrt(total_wn_plots));
num_rows_layout = ceil(total_wn_plots / num_cols_layout);

figure('Position',[100 100 1000 num_rows_layout*300]);
set(gcf, 'Name', 'Winding Number Fields');

plot_idx = 1;

% Closed Prototype - WN Field
subplot(num_rows_layout, num_cols_layout, plot_idx);
contourf(X,Y,WN_closed_reshaped,50,'LineColor','none'); hold on;
plot([edges_closed(:,1)'; edges_closed(:,3)'], [edges_closed(:,2)'; edges_closed(:,4)'],'k-','LineWidth',1.5);
axis equal tight; colorbar;
colormap(gca,custom_colormap_bwr); % 使用自定义的蓝白红色图
title('Closed Prototype WN Field');
xlabel('X'); ylabel('Y');
set(gca,'FontSize',10);
plot_idx = plot_idx + 1;

% Open Modifications - WN Fields
for k = 1:num_mods
    subplot(num_rows_layout, num_cols_layout, plot_idx);
    contourf(X,Y,WN_modified_all(:,:,k),50,'LineColor','none'); hold on;
    plot([edges_modified_all{k}(:,1)'; edges_modified_all{k}(:,3)'], ...
         [edges_modified_all{k}(:,2)'; edges_modified_all{k}(:,4)'],'r-','LineWidth',1.5);
    axis equal tight; colorbar;
    colormap(gca,custom_colormap_bwr); % 使用自定义的蓝白红色图
    title(['Open Modification ', num2str(k), ' WN Field']);
    xlabel('X'); ylabel('Y');
    set(gca,'FontSize',10);
    plot_idx = plot_idx + 1;
end

sgtitle('Winding Number Fields for All Models','FontSize',16,'FontWeight','bold');

%% --- 7. Visualization: Figure 2 - WN Threshold Classification ---
figure('Position',[150 150 1000 num_rows_layout*300]);
set(gcf, 'Name', 'Winding Number Threshold Classification');

plot_idx = 1;

% Closed Prototype - WN Threshold Classification
subplot(num_rows_layout, num_cols_layout, plot_idx);
imagesc(X(1,:),Y(:,1),WN_closed_reshaped > wn_threshold); hold on;
set(gca,'YDir','normal');
plot([edges_closed(:,1)'; edges_closed(:,3)'], [edges_closed(:,2)'; edges_closed(:,4)'],'k-','LineWidth',1.5);
axis equal tight;
% 这里可以使用一个简单的蓝白红二值色图
colormap(gca, [0.8 0.8 1; 1 0.8 0.8]); % 蓝色区域表示外部，红色区域表示内部
title(['Closed Prototype WN > ', num2str(wn_threshold)]);
xlabel('X'); ylabel('Y');
set(gca,'FontSize',10);
cb = colorbar('Ticks',[0.25, 0.75],'TickLabels',{'Outside',['Inside (WN > ',num2str(wn_threshold),')']});
cb.Label.String = 'Region';
plot_idx = plot_idx + 1;

% Open Modifications - WN Threshold Classification
for k = 1:num_mods
    subplot(num_rows_layout, num_cols_layout, plot_idx);
    imagesc(X(1,:),Y(:,1),WN_modified_all(:,:,k) > wn_threshold); hold on;
    set(gca,'YDir','normal');
    plot([edges_modified_all{k}(:,1)'; edges_modified_all{k}(:,3)'], ...
         [edges_modified_all{k}(:,2)'; edges_modified_all{k}(:,4)'],'r-','LineWidth',1.5);
    axis equal tight;
    % 同样使用蓝白红二值色图
    colormap(gca, [0.8 0.8 1; 1 0.8 0.8]); % 蓝色区域表示外部，红色区域表示内部
    title(['Open Modification ', num2str(k), ' WN > ', num2str(wn_threshold)]);
    xlabel('X'); ylabel('Y');
    set(gca,'FontSize',10);
    cb = colorbar('Ticks',[0.25, 0.75],'TickLabels',{'Outside',['Inside (WN > ',num2str(wn_threshold),')']});
    cb.Label.String = 'Region';
    plot_idx = plot_idx + 1;
end

sgtitle(['Winding Number > ', num2str(wn_threshold), ' Threshold Classification for All Models'], ...
        'FontSize',16,'FontWeight','bold');

%% --- 8. Winding Number Calculation Functions ---
function wn_field = compute_winding_number_field(query_points, edges)
    num_query_points = size(query_points,1);
    num_edges = size(edges,1);
    wn_field = zeros(num_query_points,1);
    for i = 1:num_query_points
        p = query_points(i,:);
        total_angle = 0;
        for j = 1:num_edges
            p1 = edges(j,1:2);
            p2 = edges(j,3:4);
            total_angle = total_angle + calculate_segment_winding_number(p,p1,p2);
        end
        wn_field(i) = total_angle/(2*pi);
    end
end

function angle_diff = calculate_segment_winding_number(point,p1,p2)
    v1 = p1 - point; v2 = p2 - point;
    cross_prod = v1(1)*v2(2) - v1(2)*v2(1);
    dot_prod = v1(1)*v2(1) + v1(2)*v2(2);
    if norm(v1)==0 || norm(v2)==0
        angle_diff = 0;
        return;
    end
    angle_diff = atan2(cross_prod,dot_prod);
end