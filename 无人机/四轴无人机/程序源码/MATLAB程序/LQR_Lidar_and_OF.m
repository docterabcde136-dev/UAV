clc
clear
close all
%% Parameter
m1 = 0.079 + 0.050 + 0.010; % 单位：kg
m2 = 0.014; % 单位：kg
m = m1 + 4*m2; % 单位：kg
I_xx = (1/12)*m1*(0.069^2+0.012^2) + 4*m2*0.0636^2; % 单位：kg·m²
I_yy = (1/12)*m1*(0.069^2+0.012^2) + 4*m2*0.0636^2; % 单位：kg·m²
I_zz = (1/12)*m1*(0.069^2+0.069^2) + 4*m2*0.0900^2; % 单位：kg·m²
%% Displacement(X-axis)
A = [0 1; 0 0];
B = [0; 9.8];
C = eye(2);
D = 0;
Ts = 0.005;
t = 0:Ts:10;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts); % discretizing
x0 = [-1; 0];
Tc = ctrb(G,H);
if (rank(Tc)==2)
    Q = [50 0; 0 10];
    R = 1e2;
    Kx = dlqr(G,H,Q,R);
    G2 = G-H*Kx;
    H = zeros(2,1);
    y = dlsim(G2,H,C,D,u,x0);
    subplot(3,1,1)
    plot(t,y(:,1),'b','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel("x(m)");
    grid on
end
%% Displacement(Y-axis)
A = [0 1; 0 0];
B = [0; -9.8];
C = eye(2);
D = 0;
Ts = 0.005;
t = 0:Ts:10;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts); % discretizing
x0 = [-1; 0];
Tc = ctrb(G,H);
if (rank(Tc)==2)
    Q = [50 0; 0 10];
    R = 80;
    Ky = dlqr(G,H,Q,R);
    G2 = G-H*Ky;
    H = zeros(2,1);
    y = dlsim(G2,H,C,D,u,x0);
    subplot(3,1,2)
    plot(t,y(:,1),'b','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel("y(m)");
    grid on
end
%% Height
A = [0 1; 0 0];
B = [0; 1/m];
C = eye(2);
D = 0;
Ts = 0.005;
t = 0:Ts:10;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts); % discretizing
x0 = [-1; 0];
Tc = ctrb(G,H);
if (rank(Tc)==2)
    Q = [5 0; 0 10];
    R = 15;
    K1 = dlqr(G,H,Q,R);
    G2 = G-H*K1;
    y = dlsim(G2,H,C,D,u,x0);
    subplot(3,1,3)
    plot(t,y(:,1),'b','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel("h(m)");
    grid on
end
%% Roll
A = [0 1; 0 0];
B = [0; 1/I_xx];
C = eye(2);
D = 0;
Ts = 0.005;
t = 0:Ts:10;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts); % discretizing
x0 = [0.0873; 0];
Tc = ctrb(G,H);
if (rank(Tc)==2)
    Q = [500 0; 0 10];
    R = 5e4;
    K2 = dlqr(G,H,Q,R);
    G2 = G-H*K2;
    H = zeros(2,1);
    y = dlsim(G2,H,C,D,u,x0);
    figure
    subplot(3,1,1)
    plot(t,y(:,1),'b','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel("\phi(rad)");
    grid on
end
%% Pitch
A = [0 1; 0 0];
B = [0; 1/I_yy];
C = eye(2);
D = 0;
Ts = 0.005;
t = 0:Ts:10;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts); % discretizing
x0 = [0.0873; 0];
Tc = ctrb(G,H);
if (rank(Tc)==2)
    Q = [500 0; 0 10];
    R = 5e4;
    K3 = dlqr(G,H,Q,R);
    G2 = G-H*K3;
    H = zeros(2,1);
    y = dlsim(G2,H,C,D,u,x0);
    subplot(3,1,2)
    plot(t,y(:,1),'b','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel("\theta(rad)");
    grid on
end
%% Yaw
A = [0 1; 0 0];
B = [0; 1/I_zz];
C = eye(2);
D = 0;
Ts = 0.005;
t = 0:Ts:10;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts); % discretizing
x0 = [0.0873; 0];
Tc = ctrb(G,H);
if (rank(Tc)==2)
    Q = [100 0; 0 10];
    R = 1e5;
    K4 = dlqr(G,H,Q,R);
    G2 = G-H*K4;
    H = zeros(2,1);
    y = dlsim(G2,H,C,D,u,x0);
    subplot(3,1,3)
    plot(t,y(:,1),'b','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel("\psi(rad)");
    grid on
end
