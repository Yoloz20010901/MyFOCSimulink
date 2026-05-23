% 开关频率
SwitchFrequency = 20e3;
my_Ts = 1/SwitchFrequency;   % my_Ts: 避免与Simulink内置Ts冲突
Ts = my_Ts;                  % 兼容旧模型使用Ts的块
Tpwm = 4000-1;

% mode
mode = 1;           % 0: 无感  1: 有感

vbus = 12;          % 直流母线电压 (2204电机额定12V)
vn = vbus/sqrt(2);

% 电机参数 — 2204 2300KV 12N14P
Np = 7;             % 极对数 (14极/2)
Rs = 0.112;         % 定子电阻 112mΩ
Ls = 0.000009;      % 定子电感 9μH
Ld = 0.000009;      % d轴电感
Lq = 0.000009;      % q轴电感 (表贴式 Ld≈Lq)
Flux = 0.00034;     % 磁链 0.34mWb
Jx = 0.0000005;     % 转动惯量 (先估,用x12模型辨识后替换)
delta = 4;          % 阻尼系数
Fcoef = 0;
spd_max = 27000;    % 最大机械转速 rpm
P = 120;
Te = P/(spd_max/Np);
KT = 1.5*Np*Flux;   % = 1.5*7*0.00034 = 0.00357 Nm/A

vl = 1.5;

% 电流环 PI — Ls=9μH极低,带宽从500→200
CurrentLoopBandwidth = 200*2*pi;
id_kp = Ls*CurrentLoopBandwidth;   % = 9e-6 * 1257 = 0.0113
id_ki = Rs*CurrentLoopBandwidth;   % = 0.112 * 1257 = 140.8
vd_limit = vbus/sqrt(3);

iq_kp = Ls*CurrentLoopBandwidth;
iq_ki = Rs*CurrentLoopBandwidth;
vq_limit = vbus/sqrt(3);
CurrentLoopMax = vbus/sqrt(3);

% 速度环 PI
K = (3.0*Np*Flux) / (4.0 * Jx);
spd_kp = (iq_kp/Ls)/(delta*K)/9.55;
spd_ki = ((iq_kp/Ls)*(iq_kp/Ls))/(delta*delta*delta*K);

iq_limit = 12;       % 最大电流 12A
SpdLoopMax = 12;

% 位置环 PI
pos_kp = 54;
pos_ki = 0;
speed_limit = 27000;

% smo 滑模观测器
smo_wn = 2*pi*Ts;
smo_freq = 100;
Kslf = smo_freq*smo_wn;
Fsmopos = 1.0 - Ts * Rs / Ls;
Gsmopos = Ts / Ls;
Kslide = 10000.865;
E0 = 1000.5;

smo_pll_wn = 50*2*pi;
smo_pll_kp = 2*smo_pll_wn;
smo_pll_ki = smo_pll_wn*smo_pll_wn;
smo_pll_out_max = 333*pi*2;

% 非线性磁链观测器 PLL
pll_wn = 50*2*pi;
pll_kp = 2*pll_wn;
pll_ki = pll_wn*pll_wn;
pll_out_max = 333*2*pi*2;

obs_gain = 500;
gamma_now = 10*(Flux*Flux);

% 静态电压补偿
alpha0 = 1000;

lpf_freq = 100;
lpf_coef = 1/(1+lpf_freq*6.28*Ts);

% active flux 观测器
flux_currentloop_bandwidth = 50*2*pi;
active_gain = flux_currentloop_bandwidth*Ls;
active_gain2 = 50000*Ls;

% 高频信号注入
vdh = 0.5;
% AB轴锁相环
hfsi_ab_pll_wn = 500*2*pi;
hfsi_ab_pll_kp = 2*hfsi_ab_pll_wn;
hfsi_ab_pll_ki = hfsi_ab_pll_wn*hfsi_ab_pll_wn;
hfsi_ab_pll_out_max = 500*2*pi*2;
% 速度环锁相环
hfsi_pll_wn = 50*2*pi;
hfsi_pll_kp = 2*hfsi_pll_wn;
hfsi_pll_ki = hfsi_pll_wn*hfsi_pll_wn;
hfsi_pll_out_max = 333*2*pi*2;

delta1 = 0.2;
delta_spd = 200;
inertia = (0.001079*Np)/(delta_spd*Np);

%% 弱磁控制算法V1
Ki_fw = 90;
%% 弱磁控制算法V2
fw_kp = 0.0005;
fw_ki = 0.001;
Ld_Lq = Ld-Lq;   % 表贴式=0, MTPA段计算需注意除零

%% 电压极限圆参数
i_max = 12;             % 最大电流 12A
i_s = 12;
u_max = vbus/sqrt(3);
u_s = vbus/sqrt(3);

% i_dmax: 完全去磁所需d轴电流
% 表贴式电机(Ld==Lq)此式分母为0, 跳过后面的弱磁/MTPA计算
if abs(Ld - Lq) > 1e-12
    i_dmax = Flux/Ld;
else
    i_dmax = 0;
end

%% MTPA/弱磁 临界速度计算 (仅凸极电机有效, 表贴式跳过)
if abs(Ld - Lq) > 1e-12
    i_dA = (-Flux + sqrt(Flux^2 + 8*(Ld - Lq)^2*i_s^2))/4/(Ld - Lq);
    i_qA = sqrt(i_s^2 - i_dA^2);
    T_eA = 1.5*Np*((Ld-Lq)*i_dA + Flux)*i_qA;
    w_A = u_max/sqrt((Ld*i_dA + Flux)^2+(Lq*i_qA)^2);
else
    i_dA = 0;
    i_qA = i_s;
    T_eA = 1.5*Np*Flux*i_qA;
    w_A = u_max/sqrt(Flux^2+(Lq*i_qA)^2);
end

w_B = u_max / Flux;

w_AList = linspace(4.942e+02, 800, 200);
i_dAB = [];
i_qAB = [];
if abs(Ld^2 - Lq^2) > 1e-12
    for i = 1:length(w_AList)
        i_dAB(i) = (-Flux*Ld + sqrt((Flux*Ld)^2 - (Ld^2-Lq^2)*(Flux^2 + Lq^2*i_s^2 - u_s^2/w_AList(i)^2))) / (Ld^2-Lq^2);
        i_qAB(i) = sqrt(i_s^2 - i_dAB(i)^2);
    end
end

%% MTPA公式预计算 (仅凸极电机)
Ld_Lq = Ld-Lq;
if abs(Ld - Lq) > 1e-12
    a_1 = 4*(Flux^2 - 4*(Ld-Lq)^2);
else
    a_1 = 0;
end

final_i_ref = 0;
