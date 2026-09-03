clc
clear
close all
%% 读取数据
load('data.mat');
ax = data.ax; % X方向的加速度
vx = data.vx; % X方向的速度
gy = data.gy; % 绕Y轴转动的角速度
h = data.h; % 高度
Ts = 0.005; % 采样间隔
t = 0:Ts:(size(h,2)-1)*Ts; % 时间
%% 一阶低通滤波（角速度）
% 设置滤波系数
alpha = 0.1;
%
% 初始化输出向量
gy_filtered = zeros(size(gy));
gy_filtered(1) = gy(1); % 初始化第一个输出值，避免初始条件问题
%
% 应用递推公式进行滤波
for n = 2:length(gy)
    gy_filtered(n) = alpha * gy(n) + (1 - alpha) * gy_filtered(n-1);
end
%% 旋转补偿
vx_rot_comp = zeros(1,length(vx));
vx_rot_comp(1) = (vx(1) + gy_filtered(1)) * h(1);
for n = 2:length(vx)
    if(vx(n)==vx(n-1))
        vx_rot_comp(n) = vx_rot_comp(n-1);
    else
        vx_rot_comp(n) = (vx(n) + gy_filtered(n)) * h(n);
    end
end
%% 对加速度积分得到速度
ax2vx = zeros(size(vx));
ax2vx(1) = vx(1); % 初始化第一个输出值，避免初始条件问题
%
% 积分求和
for n = 2:length(ax2vx)
    ax2vx(n) = ax2vx(n-1) + ax(n-1)*Ts;
end
%% 互补滤波
% 互补滤波参数设置
comp_alpha = 0.9; % 互补滤波系数，决定信任加速度计数据的程度
%
% 初始化互补滤波输出
vx_comp = zeros(size(vx));
vx_comp(1) = vx(1); % 初始值
%
% 应用互补滤波
for n = 2:length(vx_comp)
    % 互补滤波公式：融合积分得到的速度和旋转补偿后的速度
    vx_comp(n) = comp_alpha * (vx_comp(n-1) + ax(n)*Ts) + (1 - comp_alpha) * vx_rot_comp(n);
end
%% 计算位移
% 通过积分速度得到位移
disp_vx_rot_comp = zeros(size(vx_rot_comp));
disp_ax2vx = zeros(size(ax2vx));
disp_vx_comp = zeros(size(vx_comp));
%
% 使用梯形积分法计算位移
for n = 2:length(vx_rot_comp)
    % 旋转补偿速度的位移
    disp_vx_rot_comp(n) = disp_vx_rot_comp(n-1) + (vx_rot_comp(n) + vx_rot_comp(n-1)) * Ts / 2;
    %
    % 加速度积分的位移
    disp_ax2vx(n) = disp_ax2vx(n-1) + (ax2vx(n) + ax2vx(n-1)) * Ts / 2;
    %
    % 互补滤波速度的位移
    disp_vx_comp(n) = disp_vx_comp(n-1) + (vx_comp(n) + vx_comp(n-1)) * Ts / 2;
end
%% 绘制图像
figure('Units', 'normalized', 'Position', [0.25, 0.25, 0.50, 0.50]);
%
% 图1: 速度比较图
subplot(3,1,1);
plot(t, vx_rot_comp, 'r', 'LineWidth', 1.5);
hold on;
plot(t, ax2vx, 'b', 'LineWidth', 1.5);
plot(t, vx_comp, 'g', 'LineWidth', 1.5);
grid on;
xlim([0 max(t)]);
xlabel('time (s)');
ylabel('velocity (m/s)');
title('Velocity Comparison');
legend('Rotational Compensated', 'Acceleration Integrated', ...
       'Complementary Filtered', 'Location', 'southwest');
%
% 图2: 位移比较图
subplot(3,1,2);
plot(t, disp_vx_rot_comp, 'r', 'LineWidth', 1.5);
hold on;
plot(t, disp_ax2vx, 'b', 'LineWidth', 1.5);
plot(t, disp_vx_comp, 'g', 'LineWidth', 1.5);
grid on;
xlim([0 max(t)]);
xlabel('time (s)');
ylabel('displacement (m)');
title('Displacement Comparison');
legend('Rotational Compensated', 'Acceleration Integrated', ...
       'Complementary Filtered', 'Location', 'southwest');
%
% 图3: 互补滤波结果（速度和位移）
subplot(3,1,3);
yyaxis left;
plot(t, vx_comp, 'g', 'LineWidth', 1.5);
ylabel('velocity (m/s)');
yyaxis right;
plot(t, disp_vx_comp, 'm', 'LineWidth', 1.5);
ylabel('displacement (m)');
grid on;
xlim([0 max(t)]);
xlabel('time (s)');
title('Complementary Filter Results');
legend('Velocity', 'Displacement', 'Location', 'southeast');
