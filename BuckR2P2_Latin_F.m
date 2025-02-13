%% Clear workspace and configure display
close all;
clearvars;
clc;

% Configure Bode plot options
opts = bodeoptions('cstprefs');
opts.Title.Interpreter = 'latex';
opts.XLabel.Interpreter = 'latex';
opts.YLabel.Interpreter = 'latex';
opts.Title.FontSize = 12;
opts.XLabel.FontSize = 12;
opts.YLabel.FontSize = 12;
opts.XLabel.String = 'Frequency (Hz)';
opts.Title.String = 'Bode Diagram';
opts.FreqUnits = 'Hz';
opts.Grid = 'on';

%% Nominal parameters of the R2P2 converter
L1 = 66e-6;   % Inductor L1 [H]
L2 = 1350e-6; % Inductor L2 [H]
L3 = 1120e-6; % Inductor L3 [H]
C1 = 10e-6;   % Capacitor C1 [F]
C2 = 10e-6;   % Capacitor C2 [F]
C3 = 10e-6;   % Capacitor C3 [F]
D = 0.633;    % Duty cycle
R = 7.07;     % Load resistance [Ohm]
E = 120;      % Input voltage [V]
fs = 50e3;    % Switching frequency [Hz]

%% State-space matrices
A = [0  0  0 -1/L1  0  0;
     0  0  0  D/L2 -1/L2 -1/L2;
     0  0  0  0  D/L3 -(1-D)/L3;
     1/C1 -D/C1  0  0  0  0;
     0  1/C2 -D/C2  0  0  0;
     0  1/C3 (1-D)/C3  0  0 -1/(R*C3)];
 
B1 = [0;
      E/L2;
      (E*D)/L3;
      -(E*(D^3))/(C1*R);
      -(E*(D^2))/(C2*R);
      -(E*(D^2))/(C3*R)];

B2 = [1/L1; 0; 0; 0; 0; 0];

C6 = [0 0 0 0 0 1]; % Output selection (voltage at C3)

%% Transfer function from duty cycle to output voltage
[num, den] = ss2tf(A, B1, C6, 0);
Gvd = tf(num, den);

% Plot pole-zero map
figure;
pzmap(Gvd);
title('Pole-Zero Map of G_{vd}(s)');

figure
bode(Gvd)
title('Transfer function of G_{vd}(s)');

%% Controller parameters
V_out = 48;     % Desired output voltage [V]
F_sw = 50e3;    % Switching frequency [Hz]
Vp = 3;         % Peak voltage of the modulator
Km = 1 / Vp;    % Modulation gain
V_ref = 3;      % Reference voltage [V]
H = V_ref / V_out; % Sensor gain

% Controller design parameters
fc = 1750;       % Target crossover frequency [Hz]
wc = 2 * pi * fc; % Target crossover angular frequency [rad/s]
wz = 2 * pi * 500;   % Zero frequency [rad/s]
wp = 2 * pi * 5000;  % Pole frequency [rad/s]

% Open-loop transfer function
L_open = Gvd * Km * H;

% Compute gain at crossover frequency
s = tf('s');
[mag, ~] = bode(L_open, wc);

% Lead-lag compensator
G2 = ((s / wz + 1)^2) / (s * (s / wp + 1)^2);
[mag2, ~] = bode(G2, wc);
K = wc * (1 + (wc / wp)^2) / (mag * (1 + (wc / wz)^2));

%% Component values for controller implementation
R1x=10000
C3x=1/R1x*(1/wz-1/wp)
C1x=wz/(wp*R1x*K)
C2x=1/(R1x*K)-C1x
R2x=(C1x+C2x)/(C1x*C2x*wp)
R3x=1/(C3x*wz)-R1x

% Controller transfer function
G=(s*R2x*C2x+1)*(s*C3x*(R1x+R3x)+1)/((s*R1x*(C1x+C2x)*(1+s*R2x*(C1x*C2x)/(C1x+C2x))*(s*R3x*C3x+1)));

Hc = G;

% Extract zeros and poles in Hz
[numHc, denHc] = tfdata(Hc, 'v');
[zHc, pHc, ~] = tf2zp(numHc, denHc);
zHc_hz = zHc / (2 * pi);
pHc_hz = pHc / (2 * pi);

% Closed-loop system
L_cl = minreal(G * L_open);

%% Plot results
figure;
bode(L_cl, opts);
hold on;
bode(G, opts);
title('Bode Plot of Closed-Loop System and Controller');

figure;
nyquist(L_cl);
title('Nyquist Diagram of Open-Loop System');