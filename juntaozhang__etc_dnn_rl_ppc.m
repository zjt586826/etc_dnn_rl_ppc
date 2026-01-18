clc
close all
clear all
Time=0.01;stoptime=20;k=1;deltaT=0.012; dt=Time; %j=1; 
gamma_11=10;gamma_21=20; beta_1=1; beta_2=1;omega_1=2;omega_2=2;
gamma_12=0.001;gamma_13=0.001;gamma_22=0.001;gamma_23=0.001;p=2;q=0.9;
Gamma_1=0.4;sigma_1=3; Gamma_2=0.5;sigma_2=3;%for identifier
gamma_c1=3;gamma_a1=4; gamma_c2=3;gamma_a2=4; %correlation coefficients
rho=zeros(1);rho_3=zeros(1);
x_1=zeros(1); x_2=zeros(1); y_d=zeros(1); %states and references with ther initial values
z_1=zeros(1);xi_1=zeros(1);xi_2=zeros(1);%tracking error 
w=zeros(1,1);  dw=zeros(1,1);  %stochastic noise %stochastic for white noise
vartheta=zeros(1);varpi=zeros(1);tau=zeros(0);%design constants(performance)
C_1=zeros(1);C_2=zeros(1);x_h1=zeros(3,1);

% 事件触发参数
theta = 0.15;     % 触发阈值参数
epsilon = 0.1;   % 触发阈值常数项
k_eta = 0.25;      % 动态变量衰减率
e=zeros(1); eta = zeros(1); trigger_event = zeros(1);
% 事件触发参数
e_threshold = 0.01;        % 触发阈值
min_interval = 0.5;      % 最小触发间隔
last_trigger_time = -inf; % 上次触发时间
u_last = 0;               % 上次控制输入
xi_2_last = 0;            % 上次触发时的xi_2值
trigger_events = [];      % 触发事件记录
cost_1=zeros(1); cost_2=zeros(1);

W_a1=zeros(16,1);W_c1=zeros(16,1); S_J1=zeros(16,1);%NN weight for first subsystems
w_a1(k)=zeros(1); w_c1(k)=zeros(1); % NN weight norm for first subsystems
V_01=zeros(3,5); V_11=zeros(5,5);V_21=zeros(5,5); V_31=zeros(5,5);V_41=zeros(5,1);%DNN weight update for first subsystems (h)
W_01=zeros(15,1); W_11=zeros(25,1);W_21=zeros(25,1); W_31=zeros(25,1);W_41=zeros(5,1);%DNN weight update for first subsystems (h)的列向量表示形式
Lambda_01=zeros(15,1);Lambda_11=zeros(25,1);Lambda_21=zeros(25,1);Lambda_31=zeros(25,1);Lambda_41=zeros(5,1);
v_01(k)=zeros(1);v_11(k)=zeros(1); v_21(k)=zeros(1); v_31(k)=zeros(1); v_41(k)=zeros(1);% NN weight norm for first subsystems(h)



W_c2=zeros(12,1); W_a2=zeros(12,1);S_J2=zeros(12,1); % NN weights for second subsystems
w_c2(k)=zeros(1); w_a2(k)=zeros(1); %weigh norms for the second subsystem
V_02=zeros(4,5); V_12=zeros(5,5);V_22=zeros(5,5); V_32=zeros(5,5);V_42=zeros(5,1);%DNN weight update for  the second  subsystems (h)
W_02=zeros(20,1); W_12=zeros(25,1);W_22=zeros(25,1); W_32=zeros(25,1);W_42=zeros(5,1);%DNN weight update for the second subsystems (h)的列向量表示形式
Lambda_02=zeros(20,1);Lambda_12=zeros(25,1);Lambda_22=zeros(25,1);Lambda_32=zeros(25,1);Lambda_42=zeros(5,1);
v_02(k)=zeros(1);v_12(k)=zeros(1); v_22(k)=zeros(1); v_32(k)=zeros(1); v_42(k)=zeros(1);% NN weight norm for  the second  subsystems(h)

%第一个子系统权重更新定律V的参数
I01=eye(5);
I11=eye(5);
I21=eye(5);
I31=eye(5);
I41=eye(1);
%学习率
T01=0.4*eye(15);
T11=0.4*eye(25);
T21=0.4*eye(25);
T31=0.4*eye(25);
T41=0.4*eye(5);


%第二个子系统权重更新定律V的参数
I02=eye(5);
I12=eye(5);
I22=eye(5);
I32=eye(5);
I42=eye(1);
%学习率
T02=0.5*eye(20);
T12=0.5*eye(25);
T22=0.5*eye(25);
T32=0.5*eye(25);
T42=0.5*eye(5);

alpha=zeros(1);u=zeros(1); %the virtual and actual controller
totalCost1=zeros(1);


 

for t=0:Time:stoptime
     randn('state',100) 
   if k==1
    dw(k)=sqrt(dt)*randn;       % initial values for stochastic process
    w(k)=dw(k);                                    % W(0) = 0 不允许，所以首先置值             
   end
   if k<=stoptime/Time
    dw(k)=sqrt(dt)*randn;   % 产生序列
    w(k+1)=w(k)+dw(k); 
   end
    if k==1
        x_1(k)=2; x_2(k)=3;y_d(k)=0;rho(k)=4;rho_3(k)=-1.9;
    for i=1:16
        W_a1(i,k)=0.5; W_c1(i,k)=0.3;
    end
    for i=1:12
         W_a2(i,k)=0.3; W_c2(i,k)=0.1; 
    end
    for i=1:15
            W_01(i,1)=0.2;
    end
    for i=1:25
            W_11(i,1)=0.2;
            W_21(i,1)=0.2;
            W_31(i,1)=0.2;
    end
    for i=1:5
            W_41(i,1)=0.2;
    end
    for i=1:20
            W_02(i,1)=0.5;
    end
    for i=1:25
            W_12(i,1)=0.5;
            W_22(i,1)=0.5;
            W_32(i,1)=0.5;
    end
    for i=1:5
            W_42(i,1)=0.5;
    end
    end
    V_01= reshape(W_01(:,k), 3, 5);
    V_11= reshape(W_11(:,k), 5, 5);
    V_21= reshape(W_21(:,k), 5, 5);
    V_31= reshape(W_31(:,k), 5, 5);
    V_41= reshape(W_41(:,k), 5, 1);
    V_02= reshape(W_02(:,k), 4, 5);
    V_12= reshape(W_12(:,k), 5, 5);
    V_22= reshape(W_22(:,k), 5, 5);
    V_32= reshape(W_32(:,k), 5, 5);
    V_42= reshape(W_42(:,k), 5, 1);
 %%%%% the first backstepping step
    z_1(k)=x_1(k)-y_d(k);%define tracking error
    vartheta(k)=z_1(k)/rho(k);

    xi_1(k)=1/2*log((vartheta(k)+0.9)/(0.9-vartheta(k)));
    x_h1=[x_1(k);xi_1(k);1];
    B41=V_41'*diag(1-tanh(V_31'*tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1)))).^2);
    B31=V_31'*diag(1-(tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1)))).^2);
    B21=V_21'*diag(1-(tanh(V_11'*tanh(V_01'*x_h1))).^2);
    B11=V_11'*diag(1-(tanh(V_01'*x_h1)).^2);
    C41=tanh(V_31'*tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1))));
    C31=tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1)));
    C21=tanh(V_11'*tanh(V_01'*x_h1));
    C11=tanh(V_01'*x_h1);
    D01=kron(I01,x_h1');
    D11=kron(I11,C11');
    D21=kron(I21,C21');
    D31=kron(I31,C31');
    D41=kron(I41,C41');

    %第一个系统的参量
    Lambda_01=(B41*B31*B21*B11*D01)';
    Lambda_11=(B41*B31*B21*D11)';
    Lambda_21=(B41*B31*D21)';
    Lambda_31=(B41*D31)';
    Lambda_41=(D41)';
   %将第一个系统的所有更新权重矩阵进行按列排列
    W_01(:,k)=V_01(:);W_11(:,k)=V_11(:); W_21(:,k)=V_21(:); W_31(:,k)=V_31(:);W_41(:,k)=V_41(:);
    for i=1:16
        S_J1(i,k)= exp(-(([x_1(k);xi_1(k)]-[8;8]+[i;i])'*([x_1(k);xi_1(k)]-[8;8]+[i;i]))/2); %basis function vector
    end

    C_1=V_41'*tanh(V_31'*tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1))));%H_1函数的估
    C_2=W_a1(:,k)'*S_J1(:,k);%J_1函数的估计
    tau(k)=1/(2*rho(k))*(1/(vartheta(k)+0.9)-1/(vartheta(k)-0.9));
    tau_part = sign(tau(k)) * (abs(tau(k))^(1/3)); % 处理 tau^(1/3)
xi1_p = sign(xi_1(k)) * (abs(xi_1(k))^(4*p-3)); % 处理 xi_1^(4*p-3)
xi1_q = sign(xi_1(k)) * (abs(xi_1(k))^(4*q-3)); % 处理 xi_1^(4*q-3)

alpha(k) = -gamma_11*xi_1(k) - 3/4*tau_part*xi_1(k) - C_1 - 1/2*C_2 + ...
           z_1(k)*rho_3(k)/rho(k) - (1/(4*beta_1))*xi_1(k)^3 - ...
           gamma_12*xi1_p - gamma_13*xi1_q;%virtual controller
    if k<=stoptime/Time 
     W_01(:,k+1)=W_01(:,k)+deltaT*(T01*(tau(k).*Lambda_01*xi_1(k)^3-sigma_1*W_01(:,k)));
     W_11(:,k+1)=W_11(:,k)+deltaT*(T11*(tau(k).*Lambda_11*xi_1(k)^3-sigma_1*W_11(:,k))); 
     W_21(:,k+1)=W_21(:,k)+deltaT*(T21*(tau(k).*Lambda_21*xi_1(k)^3-sigma_1*W_21(:,k)));
     W_31(:,k+1)=W_31(:,k)+deltaT*(T31*(tau(k).*Lambda_31*xi_1(k)^3-sigma_1*W_31(:,k)));
     W_41(:,k+1)=W_41(:,k)+deltaT*(T41*(tau(k).*Lambda_41*xi_1(k)^3-sigma_1*W_41(:,k)));  
     W_c1(:,k+1)=W_c1(:,k)+deltaT*(-gamma_c1*S_J1(:,k)*S_J1(:,k)'*W_c1(:,k)); 
     W_a1(:,k+1)=W_a1(:,k)+deltaT*(-S_J1(:,k)*S_J1(:,k)'*(gamma_a1*(W_a1(:,k)-W_c1(:,k))+gamma_c1*W_c1(:,k)));
    end
    v_01(k)=norm(W_01(:,k));v_11(k)=norm(W_11(:,k)); v_21(k)=norm(W_21(:,k)); v_31(k)=norm(W_31(:,k)); v_41(k)=norm(W_41(:,k));%weight matrice norm
    w_c1(k)=norm(W_c1(:,k)); w_a1(k)=norm(W_a1(:,k));%weight matrice norm 
    cost_1(k)=xi_1(k)^2+alpha(k)^2;%cost function 
%%%%%%%%%The second backstepping step
    xi_2(k)=x_2(k)-alpha(k); % for the second subsystems
    
    x_h2=[x_1(k);x_2(k);xi_2(k);1];
    B42=V_42'*diag(1-(tanh(V_32'*tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2)))).^2));
    B32=V_32'*diag(1-(tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2)))).^2);
    B22=V_22'*diag(1-(tanh(V_12'*tanh(V_02'*x_h2))).^2);
    B12=V_12'*diag(1-(tanh(V_02'*x_h2)).^2);
    C42=tanh(V_32'*tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2))));
    C32=tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2)));
    C22=tanh(V_12'*tanh(V_02'*x_h2));
    C12=tanh(V_02'*x_h2);
    D02=kron(I02,x_h2');
    D12=kron(I12,C12');
    D22=kron(I22,C22');
    D32=kron(I32,C32');
    D42=kron(I42,C42');
    
    %第一个系统的参量
    Lambda_02=(B42*B32*B22*B12*D02)';
    Lambda_12=(B42*B32*B22*D12)';
    Lambda_22=(B42*B32*D22)';
    Lambda_32=(B42*D32)';
    Lambda_42=(D42)';
    %将第一个系统的所有更新权重矩阵进行按列排列
   
    %%% 事件触发机制 %%%
    current_time = t;
    %trigger_condition = (abs(xi_2(k) - xi_2_last) >= e_threshold || ...
                       %(current_time - last_trigger_time) >= min_interval);
    
    
    W_02(:,k)=V_02(:);W_12(:,k)=V_12(:); W_22(:,k)=V_22(:); W_32(:,k)=V_32(:);W_42(:,k)=V_42(:);
    for i=1:12
      S_J2(i,k)= exp(-(([x_1(k);x_2(k);xi_2(k)]-[6;6;6]+1/2*[i;i;i])'*([x_1(k);x_2(k);xi_2(k)]-[6;6;6]+1/2*[i;i;i]))/2);
    end
    C_3=V_42'*tanh(V_32'*tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2))));%H_2函数的估计
    C_4=W_a2(:,k)'*S_J2(:,k);%J_2函数的估计
    
    w_c2(k)=norm(W_c2(:,k)); w_a2(k)=norm(W_a2(:,k));%weight matrix norm
    v_02(k)=norm(W_02(:,k));v_12(k)=norm(W_12(:,k)); v_22(k)=norm(W_22(:,k)); v_32(k)=norm(W_32(:,k)); v_42(k)=norm(W_42(:,k));%weight matrice norm
    %if trigger_condition
    xi2_p = sign(xi_2(k)) * (abs(xi_2(k))^(4*p-3));
xi2_q = sign(xi_2(k)) * (abs(xi_2(k))^(4*q-3));

u(k) = -gamma_21*xi_2(k) - C_3 - 1/2*C_4 - (1/(4*beta_2))*xi_2(k)^3 - ...
       gamma_22*xi2_p - gamma_23*xi2_q;%actual controller
    % 事件触发机制
    e(k) = u(k) - u_last(k);
    trigger_condition = e(k)^2 >= theta*eta(k) + epsilon;
    if trigger_condition
        u_last(k) = u(k);
        last_trigger_time = current_time;
        trigger_events = [trigger_events, t];
        trigger_event(k) = 1;
    end

    cost_2(k)=xi_2(k)^2+u(k)^2;%cost function
    totalCost1(k)=cost_1(k)+cost_2(k);%total cost function
    if k<=stoptime/Time
      W_02(:,k+1)=W_02(:,k)+deltaT.*(T02*(Lambda_02*xi_2(k)^3-sigma_2*W_02(:,k)));
      W_12(:,k+1)=W_12(:,k)+deltaT.*(T12*(Lambda_12*xi_2(k)^3-sigma_2*W_12(:,k))); 
      W_22(:,k+1)=W_22(:,k)+deltaT.*(T22*(Lambda_22*xi_2(k)^3-sigma_2*W_22(:,k)));
      W_32(:,k+1)=W_32(:,k)+deltaT.*(T32*(Lambda_32*xi_2(k)^3-sigma_2*W_32(:,k)));
      W_42(:,k+1)=W_42(:,k)+deltaT.*(T42*(Lambda_42*xi_2(k)^3-sigma_2*W_42(:,k))); 
      W_c2(:,k+1)=W_c2(:,k)+deltaT*(-gamma_c2*S_J2(:,k)*S_J2(:,k)'*W_c2(:,k)); 
      W_a2(:,k+1)=W_a2(:,k)+deltaT*(-S_J2(:,k)*S_J2(:,k)'*(gamma_a2*(W_a2(:,k)-W_c2(:,k))+gamma_c2*W_c2(:,k)));
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      y_d(k+1)=5*sin(0.5*t); %refeference 
      rho(k+1)=3.8*exp(-0.5*t)+0.2;
      rho_3(k+1)=-1.9*exp(-0.5*t);
      x_1(k+1)=x_1(k)+deltaT*(x_1(k)*sin(x_1(k))+x_2(k))+cos(x_1(k))^2*dw(k);
      x_2(k+1)=x_2(k)+deltaT*(x_2(k)*cos(x_1(k))+u(k))+sin(x_1(k))*cos(x_2(k))*dw(k); 
      % 动态变量更新
        eta(k+1) = eta(k) + deltaT*(-k_eta*eta(k) + theta*e(k)^2 - epsilon);
        eta(k+1) = max(eta(k+1), 0);
        
        % 保持控制量传递
        u_last(k+1) = u_last(k);
    end
k=k+1;
end
t=0:Time:stoptime;
























%% 1.1 跟踪性能 
figure

% 定义颜色方案
ref_color = [0.8, 0.2, 0.2];      % 参考信号颜色 - 深红色
out_color = [0, 0.4, 0.6];        % 系统输出颜色 - 深蓝色
grid_color = [0.85, 0.85, 0.85];  % 网格线颜色 - 浅灰色
zoom_box_color = [1, 0, 1];   % 放大框颜色 - 深紫色
zoom_bg_color = [0.98, 0.98, 0.95]; % 放大图背景色

% 主图
t_plot = t(1:length(x_1));

% 绘制主图曲线
plot(t_plot, y_d(1:length(t_plot)), '--', ...
     'Color', ref_color, ...
     'LineWidth', 1.8);
hold on;
plot(t_plot, x_1(1:length(t_plot)), '-', ...
     'Color', out_color, ...
     'LineWidth', 1.5);

% 定义放大区域
xlim_region = [18.5, 19.5];
ylim_region = [-1, 1];

% 在主图中绘制虚线框标出放大区域
rectangle('Position', [xlim_region(1), ylim_region(1), ...
                      diff(xlim_region), diff(ylim_region)], ...
          'EdgeColor', zoom_box_color, ...
          'LineStyle', '--', ...
          'LineWidth', 1.3);

xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'normal');
legend('Reference y_d', 'System output x_1', 'Location', 'best', ...
       'FontSize', 10, 'Box', 'off');
axis([0, 20, -6, 20]);

% 网格设置 - 更精细的控制
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
hold off;

% 创建放大子图
ax_zoom = axes('Position', [0.35, 0.45, 0.3, 0.3]);

% 设置放大子图背景色
set(ax_zoom, 'Color', zoom_bg_color);

% 计算放大区域对应的索引范围
idx_zoom = t_plot >= xlim_region(1) & t_plot <= xlim_region(2);
t_zoom = t_plot(idx_zoom);
y_d_zoom = y_d(idx_zoom);
x_1_zoom = x_1(idx_zoom);

% 在放大子图中绘制数据
plot(ax_zoom, t_zoom, y_d_zoom, '--', ...
     'Color', ref_color, ...
     'LineWidth', 2);
hold on;
plot(ax_zoom, t_zoom, x_1_zoom, '-', ...
     'Color', out_color, ...
     'LineWidth', 1.7);
hold off;

% 设置放大子图的坐标轴
xlim(ax_zoom, xlim_region);
ylim(ax_zoom, ylim_region);

% 放大子图网格设置
grid(ax_zoom, 'on');
set(ax_zoom, 'GridColor', [0.7, 0.7, 0.7], ...
             'GridAlpha', 0.5, ...
             'GridLineStyle', '-.');

% 放大子图边框
box(ax_zoom, 'on');
set(ax_zoom, 'LineWidth', 1.2);

% 放大子图标签
xlabel(ax_zoom, 'Time (s)', 'FontSize', 9);

% 调整整体图形外观
set(gcf, 'Color', 'white');
set(gca, 'FontSize', 10, ...
         'Box', 'on', ...
         'LineWidth', 1.2);

% 添加轻微阴影效果到放大子图（可选）
% 创建一个稍大的背景框
annotation('rectangle', ...
    [ax_zoom.Position(1)-0.1, ax_zoom.Position(2)-0.1, ...
     ax_zoom.Position(3)+0.1, ax_zoom.Position(4)+0.1], ...
    'Color', [0.7, 0.7, 0.7], ...
    'FaceColor', 'none', ...
    'LineStyle', 'none', ...
    'FaceAlpha', 0.05);
%% 1.2 跟踪误差及性能边界 - 带区域放大效果
figure
min_len = min([length(t), length(z_1), length(rho)]);
t_plot2 = t(1:min_len);
z_1_plot = z_1(1:min_len);
rho_plot = rho(1:min_len);

% 定义颜色方案
error_color = [0, 0.3, 0.6];        % 跟踪误差颜色 - 深蓝色
bound_color = [0.85, 0.1, 0.1];     % 性能边界颜色 - 红色
fill_color = [0.95, 0.6, 0.6];      % 填充区域颜色 - 浅红色
zoom_color = [1, 0, 1];       % 放大框颜色 - 紫色
zoom_bg_color = [0.98, 0.96, 0.92]; % 放大图背景色 - 浅米色

% 绘制主图曲线
plot(t_plot2, z_1_plot, '-', 'Color', error_color, 'LineWidth', 1.3); 
hold on;
plot(t_plot2, 0.9*rho_plot, '--', 'Color', bound_color, 'LineWidth', 1.5);
plot(t_plot2, -0.9*rho_plot, '--', 'Color', bound_color, 'LineWidth', 1.5);

% 填充性能边界区域
t_fill = [t_plot2, fliplr(t_plot2)];
y_fill = [0.9*rho_plot, fliplr(-0.9*rho_plot)];
fill(t_fill, y_fill, fill_color, 'FaceAlpha', 0.15, 'EdgeColor', 'none');

% 定义放大区域（选择误差波动明显的区域）
xlim_region = [16, 18];
ylim_region = [-0.5, 0.5];

% 在主图中用虚线框标出放大区域
rectangle('Position', [xlim_region(1), ylim_region(1), ...
                      diff(xlim_region), diff(ylim_region)], ...
          'EdgeColor', zoom_color, ...
          'LineStyle', '--', ...
          'LineWidth', 1.4, ...
          'LineWidth', 1.3);

% 主图标签和设置
xlabel('Time (s)', 'FontSize', 10, 'FontWeight', 'normal');
ylabel('Tracking error \eta_1', 'FontSize', 10, 'FontWeight', 'normal');
legend('Tracking error', 'Performance bounds', ...
       'Location', 'best', 'FontSize', 9, 'Box', 'off');
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
axis([0 20 -4 4]);
hold off;

% 创建放大子图 - 放在主图的合适位置
ax_zoom = axes('Position', [0.4, 0.17, 0.3, 0.3]);

% 设置放大子图背景色
set(ax_zoom, 'Color', zoom_bg_color);

% 提取放大区域的数据
idx_zoom = t_plot2 >= xlim_region(1) & t_plot2 <= xlim_region(2);
t_zoom = t_plot2(idx_zoom);
z_1_zoom = z_1_plot(idx_zoom);
rho_zoom = rho_plot(idx_zoom);

% 在放大子图中绘制数据
plot(ax_zoom, t_zoom, z_1_zoom, '-', 'Color', error_color, 'LineWidth', 1.4);
hold on;
plot(ax_zoom, t_zoom, 0.9*rho_zoom, '--', 'Color', bound_color, 'LineWidth', 1.6);
plot(ax_zoom, t_zoom, -0.9*rho_zoom, '--', 'Color', bound_color, 'LineWidth', 1.6);

% 填充放大区域的性能边界
t_fill_zoom = [t_zoom, fliplr(t_zoom)];
y_fill_zoom = [0.9*rho_zoom, fliplr(-0.9*rho_zoom)];
fill(ax_zoom, t_fill_zoom, y_fill_zoom, fill_color, ...
     'FaceAlpha', 0.25, 'EdgeColor', 'none');
hold off;

% 设置放大子图的坐标轴
xlim(ax_zoom, xlim_region);
ylim(ax_zoom, ylim_region);

% 放大子图网格和样式设置
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');

% 调整子图整体外观
set(gca, 'FontSize', 10, 'Box', 'on', 'LineWidth', 1.2);
%% 1.3 控制输入与事件触发 - 带区域放大效果
figure
min_len3 = min([length(t), length(u_last)]);
t_plot3 = t(1:min_len3);
u_last_plot = u_last(1:min_len3);

% 定义颜色方案
control_color = [0, 0.4, 0.2];          % 控制输入颜色 - 深绿色
trigger_color = [0.9, 0.2, 0.2];        % 触发事件颜色 - 红色
zoom_color = [0.5, 0.1, 0.7];           % 放大框颜色 - 深紫色
zoom_bg_color = [0.96, 0.96, 0.98];     % 放大图背景色 - 浅蓝灰色

% 绘制控制输入（阶梯图）
stairs(t_plot3, u_last_plot, '-', 'Color', control_color, 'LineWidth', 1.3);
hold on;

% 提取并绘制触发事件
if ~isempty(trigger_events)
    trigger_indices = zeros(1, length(trigger_events));
    for i = 1:length(trigger_events)
        [~, idx] = min(abs(t - trigger_events(i)));
        trigger_indices(i) = min(idx, min_len3);
    end
    trigger_indices = unique(trigger_indices);
    trigger_times = t_plot3(trigger_indices);
    trigger_values = u_last_plot(trigger_indices);
    
    % 绘制触发事件点
    plot(trigger_times, trigger_values, '*', ...
         'Color', trigger_color, ...
         'MarkerSize', 3, ...
         'LineWidth', 1, ...
         'MarkerFaceColor', trigger_color);
end

% 定义放大区域（选择包含多个触发事件的区域）
xlim_region = [14, 16];
% 自动确定Y轴放大范围
region_idx = t_plot3 >= xlim_region(1) & t_plot3 <= xlim_region(2);
if any(region_idx)
    y_data = u_last_plot(region_idx);
    y_min = min(y_data) - 0.5;
    y_max = max(y_data) + 0.5;
else
    y_min = min(u_last_plot) - 0.5;
    y_max = max(u_last_plot) + 0.5;
end
ylim_region = [y_min, y_max];

% 在主图中用虚线框标出放大区域
rectangle('Position', [xlim_region(1), ylim_region(1), ...
                      diff(xlim_region), diff(ylim_region)], ...
          'EdgeColor', zoom_color, ...
          'LineStyle', '--', ...
          'LineWidth', 1.4);

% 主图标签和设置
xlabel('Time (s)', 'FontSize', 10, 'FontWeight', 'normal');
ylabel('Control input u', 'FontSize', 10, 'FontWeight', 'normal');
% 修改：将图例位置改为右上角 (northeast)
legend('Control input', 'Trigger events', ...
       'Location', 'northeast', 'FontSize', 9, 'Box', 'off');
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
axis([0 20 min(u_last_plot)-1 max(u_last_plot)+1]); % 自动调整Y轴范围
axis([0 20 -250 140]);
hold off;

% 创建放大子图 - 放在主图的合适位置
% 调整放大子图位置以避免遮挡图例
ax_zoom = axes('Position', [0.53, 0.22, 0.3, 0.3]);

% 设置放大子图背景色
set(ax_zoom, 'Color', zoom_bg_color);

% 提取放大区域的数据
zoom_idx = t_plot3 >= xlim_region(1) & t_plot3 <= xlim_region(2);
t_zoom = t_plot3(zoom_idx);
u_zoom = u_last_plot(zoom_idx);

% 在放大子图中绘制阶梯图
stairs(ax_zoom, t_zoom, u_zoom, '-', 'Color', control_color, 'LineWidth', 1.5);

% 在放大子图中绘制触发事件
if ~isempty(trigger_events)
    % 提取放大区域内的触发事件
    trigger_in_zoom = trigger_times >= xlim_region(1) & trigger_times <= xlim_region(2);
    if any(trigger_in_zoom)
        trigger_times_zoom = trigger_times(trigger_in_zoom);
        trigger_values_zoom = trigger_values(trigger_in_zoom);
        
        hold(ax_zoom, 'on');
        plot(ax_zoom, trigger_times_zoom, trigger_values_zoom, '*', ...
             'Color', trigger_color, ...
             'MarkerSize', 9, ...
             'LineWidth', 1.6, ...
             'MarkerFaceColor', trigger_color);
        hold(ax_zoom, 'off');
    end
end

% 设置放大子图的坐标轴
xlim(ax_zoom, xlim_region);
ylim(ax_zoom, ylim_region);

% 放大子图网格和样式设置
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');

% 为放大子图添加X轴标签
xlabel(ax_zoom, 'Time (s)', 'FontSize', 8);

% 调整主图外观
set(gca, 'FontSize', 10, 'Box', 'on', 'LineWidth', 1.2);
%% ============ 2. 代价函数分析 ============
figure('Position', [100, 100, 800, 900]); % 设置图形大小

% 子图1：对数刻度成本函数
subplot(3, 1, 1);
% 确保所有向量长度一致
min_len = min([length(t), length(cost_1), length(cost_2), length(totalCost1)]);
t_plot = t(1:min_len);
cost_1_plot = cost_1(1:min_len);
cost_2_plot = cost_2(1:min_len);
totalCost_plot = totalCost1(1:min_len);

% 绘制对数刻度图
h1 = semilogy(t_plot, cost_1_plot, 'b-', 'LineWidth', 1.8);
hold on;
h2 = semilogy(t_plot, cost_2_plot, 'r-', 'LineWidth', 1.8);
h3 = semilogy(t_plot, totalCost_plot, 'g-', 'LineWidth', 2.2);

% 科研美化设置
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Cost (log scale)', 'FontSize', 12, 'FontWeight', 'bold');
title('Cost Function Evolution (Log Scale)', 'FontSize', 14, 'FontWeight', 'bold');
legend([h1, h2, h3], {'Subsystem 1', 'Subsystem 2', 'Total'}, 'Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 11, 'FontName', 'Arial');
ylim([1e-3, 1e4]);
xlim([0, max(t_plot)]);

% 添加网格线增强可读性
set(gca, 'YMinorGrid', 'on', 'XMinorGrid', 'on');
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.3);

% 子图2：子系统1成本函数（线性尺度）
subplot(3, 1, 2);
h4 = plot(t_plot, cost_1_plot, 'b-', 'LineWidth', 1.5);

% 科研美化设置
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Cost', 'FontSize', 12, 'FontWeight', 'bold');
title('Subsystem 1: Cost Function c_1(\eta_1,\xi_1)', 'FontSize', 13, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontName', 'Arial');
ylim([0, 200]);
xlim([0, 20]);

% 添加背景色区域突出重要部分
hold on;
y_limits = ylim;
fill([5, 15, 15, 5], [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], ...
     [0.9, 0.95, 1], 'EdgeColor', 'none', 'FaceAlpha', 0.2);

% 在子图2中添加图例
legend('c_1(\eta_1,\xi_1)', 'Location', 'best', 'FontSize', 10);

% 子图3：子系统2成本函数（线性尺度）
subplot(3, 1, 3);
h5 = plot(t_plot, cost_2_plot, 'r-', 'LineWidth', 1.5);

% 科研美化设置
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Cost', 'FontSize', 12, 'FontWeight', 'bold');
title('Subsystem 2: Cost Function c_2(\eta_2,u)', 'FontSize', 13, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'FontName', 'Arial');
ylim([0, 2000]);
xlim([0, 20]);

% 添加背景色区域
hold on;
y_limits = ylim;
fill([5, 15, 15, 5], [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], ...
     [1, 0.95, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.2);

% 添加水平参考线（可根据需要调整）
line([0, 20], [500, 500], 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--', 'LineWidth', 0.8);
line([0, 20], [1000, 1000], 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--', 'LineWidth', 0.8);

% 在子图3中添加图例
legend('c_2(\eta_2,u)', 'Location', 'best', 'FontSize', 10);

% 调整子图间距
set(gcf, 'Color', 'w'); % 设置背景为白色
ha = findobj(gcf, 'type', 'axes');
for i = 1:length(ha)
    ha(i).Position(4) = ha(i).Position(4) * 0.85; % 调整高度
end

% 调整布局
h = findobj(gcf, 'type', 'axes');
set(h, 'TickDir', 'out', 'LineWidth', 1.2);

%% ============ 4. 神经网络权重收敛性 

% 数据准备
weights1 = {v_01, v_11, v_21, v_31, v_41};
labels1 = {'v_{01}', 'v_{11}', 'v_{21}', 'v_{31}', 'v_{41}'};
colors = lines(5); % 使用MATLAB的lines配色方案

% 定义放大区域颜色
zoom_color = [0.7, 0.2, 0.8];       % 放大框颜色 - 紫色
zoom_bg_color = [0.98, 0.96, 0.92]; % 放大图背景色

% 确定最小长度以确保所有向量长度一致
min_len = min([length(t), cellfun(@length, weights1)]);
t_plot = t(1:min_len);

% 定义放大区域（选择权重收敛的后期阶段）
if max(t_plot) >= 10
    zoom_time_start = max(t_plot) * 0.7;  % 最后30%的时间开始
    zoom_time_end = max(t_plot);          % 结束时间
else
    zoom_time_start = max(t_plot) * 0.5;  % 最后50%的时间开始
    zoom_time_end = max(t_plot);          % 结束时间
end

zoom_time_range = [zoom_time_start, zoom_time_end];


figure('Position', [100, 100, 1500, 800]);

% 定义子图间距
spacing_h = 0.08;  % 水平间距
spacing_v = 0.1;   % 垂直间距
margin = [0.08, 0.05, 0.12, 0.05]; % [左, 右, 上, 下]边距

% 计算每个子图的宽度和高度
plot_width = (1 - margin(1) - margin(2) - 2*spacing_h) / 3;
plot_height = (1 - margin(3) - margin(4) - 1*spacing_v) / 2;

% 为每个子图计算位置并创建坐标轴
ax_handles = cell(6, 1);
for i = 1:6
    % 计算行和列
    row = ceil(i/3);
    col = mod(i-1, 3) + 1;
    
    % 计算位置
    left = margin(1) + (col-1)*(plot_width + spacing_h);
    bottom = margin(4) + (2-row)*(plot_height + spacing_v);
    
    % 创建坐标轴
    ax_handles{i} = axes('Position', [left, bottom, plot_width, plot_height]);
    
    % 设置基本属性
    hold(ax_handles{i}, 'on');
    grid(ax_handles{i}, 'on');
    box(ax_handles{i}, 'on');
end

% ============ 子图1-5：单独权重范数 ============
for i = 1:5
    % 设置当前坐标轴
    axes(ax_handles{i});
    
    % 提取数据
    w_data = weights1{i}(1:min_len);
    
    % 绘制主曲线
    plot(t_plot, w_data, 'Color', colors(i,:), 'LineWidth', 2, ...
         'DisplayName', labels1{i});

    % 确定放大区域的Y轴范围
    zoom_idx = t_plot >= zoom_time_range(1) & t_plot <= zoom_time_range(2);
    if sum(zoom_idx) > 0
        zoom_data = w_data(zoom_idx);
        y_min_zoom = min(zoom_data) * 0.99;
        y_max_zoom = max(zoom_data) * 1.01;
        if y_max_zoom <= y_min_zoom
            y_min_zoom = min(w_data) * 0.98;
            y_max_zoom = max(w_data) * 1.02;
        end
    else
        y_min_zoom = min(w_data) * 0.98;
        y_max_zoom = max(w_data) * 1.02;
    end
    
    % 确保高度为正数
    rect_height = y_max_zoom - y_min_zoom;
    if rect_height <= 0
        rect_height = (max(w_data) - min(w_data)) * 0.1;
        if rect_height <= 0
            rect_height = 0.1;
        end
    end
    
    % 计算矩形宽度
    rect_width = zoom_time_range(2) - zoom_time_range(1);
    if rect_width <= 0
        rect_width = max(t_plot) * 0.05;
    end
    
    % 在主图中标出放大区域
    rectangle('Position', [zoom_time_range(1), y_min_zoom, rect_width, rect_height], ...
              'EdgeColor', zoom_color, ...
              'LineStyle', ':', ...
              'LineWidth', 1.5);
    
    % 设置子图样式
    set(gca, 'FontSize', 11, 'FontName', 'Arial', 'LineWidth', 1.2);

    
    % 设置合适的Y轴范围
    y_range = [min(w_data)*0.95, max(w_data)*1.05];
    if diff(y_range) > 0
        ylim(y_range);
    end
    
    % 添加网格
    grid on; grid minor;
    
    % 图例（简洁版）
    legend('Location', 'best', 'FontSize', 9, 'Box', 'off');
    xlabel('Time (s)', 'FontSize', 10);
    hold off;
    
    % 为当前子图创建内部放大视图
    % 获取当前子图位置
    subplot_pos = get(gca, 'Position');
    
    % 计算放大图在子图内部的位置（右上角）
    zoom_ax_pos = [subplot_pos(1) + subplot_pos(3) * 0.55, ...
                   subplot_pos(2) + subplot_pos(4) * 0.55, ...
                   subplot_pos(3) * 0.35, ...
                   subplot_pos(4) * 0.35];
    
    ax_zoom = axes('Position', zoom_ax_pos);
    set(ax_zoom, 'Color', zoom_bg_color, 'Box', 'on', 'LineWidth', 1);
    
    % 提取放大区域数据
    t_zoom = t_plot(zoom_idx);
    w_zoom = w_data(zoom_idx);
    
    % 绘制放大图
    plot(ax_zoom, t_zoom, w_zoom, '-', 'Color', colors(i,:), 'LineWidth', 2);
    hold on;
    
    hold off;
    
    % 设置放大图样式
    xlim(ax_zoom, zoom_time_range);
    ylim(ax_zoom, [y_min_zoom, y_max_zoom]);
    grid(ax_zoom, 'on');
    grid(ax_zoom, 'minor');
    set(ax_zoom, 'FontSize', 8, 'LineWidth', 0.8, ...
                'GridColor', [0.85, 0.85, 0.85], ...
                'GridAlpha', 0.4, ...
                'FontName', 'Arial');
end

% ============ 子图6：所有权重综合图 ============
% 设置当前坐标轴
axes(ax_handles{6});

hold on;

% 使用与子图1-5相同的颜色绘制所有权重
for i = 1:5
    w_plot = weights1{i}(1:min_len);
    % 使用相同的颜色方案
    plot(t_plot, w_plot, 'Color', colors(i,:), 'LineWidth', 2, ...
         'DisplayName', labels1{i});
end

xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Weight Norms', 'FontSize', 12, 'FontWeight', 'bold');
title('All Weight Norms of Subsystem 1', 'FontSize', 13, 'FontWeight', 'bold');
legend(labels1, 'Location', 'best', 'FontSize', 10, 'Box', 'off');
grid on; grid minor;
set(gca, 'FontSize', 11, 'FontName', 'Arial', 'LineWidth', 1.2);
hold off;

%% ============ 4.2 第二子系统权重范数 
figure('Position', [100, 100, 1500, 800]); % 

% 数据准备
weights2 = {v_02, v_12, v_22, v_32, v_42};
labels2 = {'v_{02}', 'v_{12}', 'v_{22}', 'v_{32}', 'v_{42}'};
colors = lines(5); % 使用MATLAB的lines配色方案

% 定义放大区域颜色
zoom_color = [0.2, 0.6, 0.8];        % 放大框颜色 - 蓝色调
zoom_bg_color = [0.96, 0.98, 0.96];  % 放大图背景色 - 浅绿色调

% 确定最小长度以确保所有向量长度一致
min_len8 = min([length(t), cellfun(@length, weights2)]);
t_plot8 = t(1:min_len8);

% 定义放大区域（选择权重收敛的后期阶段）
if max(t_plot8) >= 10
    zoom_time_start = max(t_plot8) * 0.7;  % 最后30%的时间开始
    zoom_time_end = max(t_plot8);          % 结束时间
else
    zoom_time_start = max(t_plot8) * 0.5;  % 最后50%的时间开始
    zoom_time_end = max(t_plot8);          % 结束时间
end

zoom_time_range = [zoom_time_start, zoom_time_end];

% 定义子图间距
spacing_h = 0.08;  % 水平间距
spacing_v = 0.1;   % 垂直间距
margin = [0.08, 0.05, 0.12, 0.05]; % [左, 右, 上, 下]边距

% 计算每个子图的宽度和高度
plot_width = (1 - margin(1) - margin(2) - 2*spacing_h) / 3;
plot_height = (1 - margin(3) - margin(4) - 1*spacing_v) / 2;

% 为每个子图计算位置并创建坐标轴
ax_handles2 = cell(6, 1);
for i = 1:6
    % 计算行和列
    row = ceil(i/3);
    col = mod(i-1, 3) + 1;
    
    % 计算位置
    left = margin(1) + (col-1)*(plot_width + spacing_h);
    bottom = margin(4) + (2-row)*(plot_height + spacing_v);
    
    % 创建坐标轴
    ax_handles2{i} = axes('Position', [left, bottom, plot_width, plot_height]);
    
    % 设置基本属性
    hold(ax_handles2{i}, 'on');
    grid(ax_handles2{i}, 'on');
    box(ax_handles2{i}, 'on');
end

% ============ 子图1-5：单独权重范数 ============
for i = 1:5
    % 设置当前坐标轴
    axes(ax_handles2{i});
    
    % 提取数据
    w_data = weights2{i}(1:min_len8);
    
    % 绘制主曲线
    plot(t_plot8, w_data, 'Color', colors(i,:), 'LineWidth', 2, ...
         'DisplayName', labels2{i});
    
    % 确定放大区域的Y轴范围
    zoom_idx = t_plot8 >= zoom_time_range(1) & t_plot8 <= zoom_time_range(2);
    if sum(zoom_idx) > 0
        zoom_data = w_data(zoom_idx);
        y_min_zoom = min(zoom_data) * 0.99;
        y_max_zoom = max(zoom_data) * 1.01;
        if y_max_zoom <= y_min_zoom
            y_min_zoom = min(w_data) * 0.98;
            y_max_zoom = max(w_data) * 1.02;
        end
    else
        y_min_zoom = min(w_data) * 0.98;
        y_max_zoom = max(w_data) * 1.02;
    end
    
    % 确保高度为正数
    rect_height = y_max_zoom - y_min_zoom;
    if rect_height <= 0
        rect_height = (max(w_data) - min(w_data)) * 0.1;
        if rect_height <= 0
            rect_height = 0.1;
        end
    end
    
    % 计算矩形宽度
    rect_width = zoom_time_range(2) - zoom_time_range(1);
    if rect_width <= 0
        rect_width = max(t_plot8) * 0.05;
    end
    
    % 在主图中标出放大区域
    rectangle('Position', [zoom_time_range(1), y_min_zoom, rect_width, rect_height], ...
              'EdgeColor', zoom_color, ...
              'LineStyle', ':', ...
              'LineWidth', 1.5);
    
    % 设置子图样式
    set(gca, 'FontSize', 11, 'FontName', 'Arial', 'LineWidth', 1.2);
    xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 设置合适的Y轴范围
    y_range = [min(w_data)*0.95, max(w_data)*1.05];
    if diff(y_range) > 0
        ylim(y_range);
    end
    
    % 添加网格
    grid on; grid minor;
    
    % 图例（简洁版）
    legend('Location', 'best', 'FontSize', 9, 'Box', 'off');
    
    hold off;
    
    % 为当前子图创建内部放大视图
    % 获取当前子图位置
    subplot_pos = get(gca, 'Position');
    
    % 计算放大图在子图内部的位置（右上角）
    zoom_ax_pos = [subplot_pos(1) + subplot_pos(3) * 0.55, ...
                   subplot_pos(2) + subplot_pos(4) * 0.55, ...
                   subplot_pos(3) * 0.35, ...
                   subplot_pos(4) * 0.35];
    
    ax_zoom = axes('Position', zoom_ax_pos);
    set(ax_zoom, 'Color', zoom_bg_color, 'Box', 'on', 'LineWidth', 1);
    
    % 提取放大区域数据
    t_zoom = t_plot8(zoom_idx);
    w_zoom = w_data(zoom_idx);
    
    % 绘制放大图
    plot(ax_zoom, t_zoom, w_zoom, '-', 'Color', colors(i,:), 'LineWidth', 2);
    hold on;
    
    hold off;
    
    % 设置放大图样式
    xlim(ax_zoom, zoom_time_range);
    ylim(ax_zoom, [y_min_zoom, y_max_zoom]);
    grid(ax_zoom, 'on');
    grid(ax_zoom, 'minor');
    set(ax_zoom, 'FontSize', 8, 'LineWidth', 0.8, ...
                'GridColor', [0.85, 0.85, 0.85], ...
                'GridAlpha', 0.4, ...
                'FontName', 'Arial');
    
end

% ============ 子图6：所有权重综合图 ============
% 设置当前坐标轴
axes(ax_handles2{6});

% 使用相同的颜色绘制所有权重
hold on;

for i = 1:5
    w_plot = weights2{i}(1:min_len8);
    % 使用与子图1-5相同的颜色
    plot(t_plot8, w_plot, 'Color', colors(i,:), 'LineWidth', 2, ...
         'DisplayName', labels2{i});
end

xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Weight Norms', 'FontSize', 12, 'FontWeight', 'bold');
title('All Weight Norms of Subsystem 2', 'FontSize', 13, 'FontWeight', 'bold');
legend(labels2, 'Location', 'best', 'FontSize', 10, 'Box', 'off');
grid on; grid minor;
set(gca, 'FontSize', 11, 'FontName', 'Arial', 'LineWidth', 1.2);
hold off;
%% ============ 3. 事件触发条件分析 - 带区域放大效果 ============
figure
min_len5 = min([length(t), length(e), length(eta)]);
t_plot5 = t(1:min_len5);
e_plot = e(1:min_len5);
eta_plot = eta(1:min_len5);

% 绘制主图曲线
plot(t_plot5, e_plot.^2, 'b-', 'LineWidth', 1.2);
hold on;
plot(t_plot5, theta*eta_plot + epsilon, 'r--', 'LineWidth', 1.5);
plot(t_plot5, eta_plot, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)', 'FontSize', 10);
% 修改：将图例位置改为右上角 (northeast)
legend('e^2', 'Threshold θη+ε', 'Dynamic variable η', 'Location', 'northeast');
grid on;

% 定义放大区域（可根据具体数据调整）
xlim_zoom = [1, 2];  % 放大时间区间
% 计算纵坐标范围（基于放大区域内的数据）
idx_zoom = t_plot5 >= xlim_zoom(1) & t_plot5 <= xlim_zoom(2);
ylim_zoom = [min([min(e_plot(idx_zoom).^2), min(theta*eta_plot(idx_zoom)+epsilon), min(eta_plot(idx_zoom))]), ...
             max([max(e_plot(idx_zoom).^2), max(theta*eta_plot(idx_zoom)+epsilon), max(eta_plot(idx_zoom))])];
% 添加一些边距
ylim_zoom = [ylim_zoom(1) - 0.1*diff(ylim_zoom), ylim_zoom(2) + 0.1*diff(ylim_zoom)];

% 在主图上绘制放大区域框
rectangle('Position', [xlim_zoom(1), ylim_zoom(1), ...
                      diff(xlim_zoom), diff(ylim_zoom)], ...
          'EdgeColor', [1, 0, 1], 'LineStyle', '--', 'LineWidth', 1.2);

% 创建放大子图
ax_zoom = axes('Position', [0.5, 0.3, 0.4, 0.4]);  % 调整位置
set(ax_zoom, 'Color', [0.98, 0.98, 0.95], 'LineWidth', 1.2);

% 在放大图中绘制曲线
plot(ax_zoom, t_plot5(idx_zoom), e_plot(idx_zoom).^2, 'b-', 'LineWidth', 1.5);
hold on;
plot(ax_zoom, t_plot5(idx_zoom), theta*eta_plot(idx_zoom) + epsilon, 'r--', 'LineWidth', 1.8);
plot(ax_zoom, t_plot5(idx_zoom), eta_plot(idx_zoom), 'g-', 'LineWidth', 1.5);
hold off;

% 设置放大图坐标轴
xlim(ax_zoom, xlim_zoom);
ylim(ax_zoom, ylim_zoom);
grid(ax_zoom, 'on');
xlabel(ax_zoom, 'Time (s)', 'FontSize', 8);

% 添加放大图边框和背景效果
box(ax_zoom, 'on');
% 添加轻微阴影
annotation('rectangle', ...
    [ax_zoom.Position(1)-0.015, ax_zoom.Position(2)-0.015, ...
     ax_zoom.Position(3)+0.03, ax_zoom.Position(4)+0.03], ...
    'Color', [0.7, 0.7, 0.7], 'FaceColor', 'none', ...
    'LineStyle', 'none', 'FaceAlpha', 0.05);
%% AC
% 设置图形样式
set(groot, 'defaultAxesFontName', 'Arial', ...
           'defaultAxesFontSize', 10, ...
           'defaultLineLineWidth', 1.5);

% 创建包含四个子图的单个图形
figure('Position', [100, 100, 1000, 700]);  % 设置图形大小和位置

% 第1个子图：第1步Critic网络权重
subplot(2, 2, 1);
h1 = plot(t, w_c1(:), 'Color', [0, 0.447, 0.741], 'LineWidth', 1.8);  % 深蓝色
grid on;
box on;
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('Critic Network Weight (Step 1)', ...
      'FontSize', 12, 'FontWeight', 'bold', ...
      'Color', [0.15, 0.15, 0.15]);
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
xlim([min(t), max(t)]);

% 第2个子图：第1步Actor网络权重
subplot(2, 2, 2);
h2 = plot(t, w_a1(:), 'Color', [0.851, 0.325, 0.098], 'LineWidth', 1.8);  % 橙色
grid on;
box on;
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('Actor Network Weight (Step 1)', ...
      'FontSize', 12, 'FontWeight', 'bold', ...
      'Color', [0.15, 0.15, 0.15]);
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
xlim([min(t), max(t)]);

% 第3个子图：第2步Critic网络权重
subplot(2, 2, 3);
h3 = plot(t, w_c2(:), 'Color', [0.466, 0.674, 0.188], 'LineWidth', 1.8);  % 绿色
grid on;
box on;
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('Critic Network Weight (Step 2)', ...
      'FontSize', 12, 'FontWeight', 'bold', ...
      'Color', [0.15, 0.15, 0.15]);
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
xlim([min(t), max(t)]);

% 第4个子图：第2步Actor网络权重
subplot(2, 2, 4);
h4 = plot(t, w_a2(:), 'Color', [0.494, 0.184, 0.556], 'LineWidth', 1.8);  % 紫色
grid on;
box on;
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('Actor Network Weight (Step 2)', ...
      'FontSize', 12, 'FontWeight', 'bold', ...
      'Color', [0.15, 0.15, 0.15]);
grid on;
grid minor;
set(gca, 'GridColor', grid_color, ...
         'GridAlpha', 1, ...
         'MinorGridColor', grid_color, ...
         'MinorGridAlpha', 1, ...
         'GridLineStyle', '--', ...
         'MinorGridLineStyle', ':');
xlim([min(t), max(t)]);


%% 触发事件时间线 
figure('Position', [100, 100, 1500, 500]); 
subplot(1, 2, 1);

if ~isempty(trigger_events)
    % 创建更美观的stem图
    h = stem(trigger_events, ones(size(trigger_events)), ...
        'LineWidth', 1.5, ...
        'Marker', 'o', ...
        'MarkerFaceColor', [0.2, 0.4, 0.8], ... % 更深的蓝色
        'MarkerEdgeColor', [0.1, 0.3, 0.7], ...
        'MarkerSize', 6, ...
        'Color', [0.3, 0.5, 0.9], ... % 线条颜色
        'DisplayName', 'Trigger Events'); % 添加显示名称
    
    % 添加垂直线条填充效果
    hold on;
    for i = 1:length(trigger_events)
        plot([trigger_events(i), trigger_events(i)], [0, 1], ...
            'Color', [0.3, 0.5, 0.9, 0.3], ... % 半透明的垂直线
            'LineWidth', 0.5, ...
            'HandleVisibility', 'off'); % 隐藏图例
    end
    
    % 设置坐标轴
    xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2, 0.2, 0.2]);
    ylabel('Trigger Event', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2, 0.2, 0.2]);
    
    % 设置坐标轴范围
    ylim([0.8, 1.2]);
    xlim([0, stoptime]);
    
    % 美化坐标轴
    set(gca, ...
        'Box', 'on', ...
        'LineWidth', 1, ...
        'FontSize', 10, ...
        'GridColor', [0.85, 0.85, 0.85], ...
        'GridAlpha', 0.8, ...
        'MinorGridColor', [0.9, 0.9, 0.9], ...
        'XColor', [0.3, 0.3, 0.3], ...
        'YColor', [0.3, 0.3, 0.3], ...
        'YTick', [], ... % 隐藏Y轴刻度
        'Color', [0.98, 0.98, 0.98]); % 设置坐标轴背景色
    
    % 添加网格线
    grid on;
    grid minor;
    
    % 添加图例（只有一个）
    legend('Location', 'best', 'FontSize', 10);
    
    % 添加时间轴指示
    if length(trigger_events) > 1
        plot([0, stoptime], [1, 1], 'k--', 'LineWidth', 0.5, ...
            'Color', [0.7, 0.7, 0.7], ...
            'HandleVisibility', 'off'); % 隐藏图例
    end
    
    hold off;
    
else
    % 美化无事件显示
    axes('Position', [0.1, 0.1, 0.8, 0.8], 'Color', [0.95, 0.95, 0.95]);
    text(0.5, 0.5, 'No Trigger Events Recorded', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 14, ...
        'FontWeight', 'bold', ...
        'Color', [0.5, 0.5, 0.5]);
    
    text(0.5, 0.4, 'Data collection period is active', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 11, ...
        'FontAngle', 'italic', ...
        'Color', [0.7, 0.7, 0.7]);
    
    axis off;
end

%% 绘制Interevent Time图

% 确保有触发事件
if ~isempty(trigger_events)
    % 提取触发时间
    trigger_times = trigger_events;
    
    % 计算相邻触发事件的时间间隔
    interevent_times = diff(trigger_times);
    
    % 计算触发事件的序号
    event_numbers = 1:length(interevent_times);
    
    subplot(1, 2, 2);
    stem(event_numbers, interevent_times, 'filled', 'LineWidth', 0.5);
    xlabel('Event Number (k)');
    ylabel('Interevent Time $t_{k+1} - t_k$ (s)', 'Interpreter', 'latex');
    grid on;
    
    
else
    fprintf('没有触发事件发生。\n');
end
%% ============ 10. 性能统计表格输出 ============
fprintf('\n=========== 性能统计摘要 ===========\n');
fprintf('仿真时间: %.1f s\n', stoptime);
fprintf('总仿真步数: %d\n', length(t));
fprintf('触发次数: %d\n', length(trigger_events));
fprintf('触发率: %.2f%%\n', 100*length(trigger_events)/length(t));

if ~isempty(trigger_events) && length(trigger_events) > 1
    trigger_intervals = diff(trigger_events);
    fprintf('平均触发间隔: %.4f s\n', mean(trigger_intervals));
    fprintf('最小触发间隔: %.4f s\n', min(trigger_intervals));
    fprintf('最大触发间隔: %.4f s\n', max(trigger_intervals));
end

% 计算RMS误差
rms_error = sqrt(mean(z_1(1:min_len).^2));
fprintf('RMS跟踪误差: %.4f\n', rms_error);

% 平均控制能量
avg_control = mean(abs(u_last(1:min_len3)));
fprintf('平均控制能量: %.4f\n', avg_control);

% 最大跟踪误差
max_error = max(abs(z_1(1:min_len)));
fprintf('最大跟踪误差: %.4f\n', max_error);

% 计算是否违反性能边界
violation_indices = abs(z_1_plot) > 0.9*rho_plot;
violation_rate = sum(violation_indices) / length(z_1_plot);
fprintf('性能边界违反率: %.2f%%\n', 100*violation_rate);

% 性能指标总结
performance_metrics = struct(...
    'TotalTime', stoptime, ...
    'TotalSteps', length(t), ...
    'TriggerEvents', length(trigger_events), ...
    'TriggerRate', 100*length(trigger_events)/length(t), ...
    'RMSError', rms_error, ...
    'AvgControl', avg_control, ...
    'MaxError', max_error, ...
    'BoundViolationRate', 100*violation_rate);

fprintf('\n=========== 总结 ===========\n');
disp(struct2table(performance_metrics, 'AsArray', true));