% Richard Chen, Nathan Dabling and Fangzhong Guo 
% Capstone Design
% Week 5 Spr

clear all
close all
clc

%% System Requirements
M = 1;          % Mass
OS = 10;        % Percent Overshoot
B = .00;       % Bearing Damping
J = .001;       % Wheel and Motor inertia
Km = .058;       % Motor Constant (Taken from ME 471 DC Motor Lab)
Ka = .79;       % Current Amp. Gain (Taken from ME 471 DC Motor Lab)
r = .0176/2;        % Wheel Radius
c = 2*pi*r;
N = 5.9;        % Gear Ratio

Rmot = 4;
zeta = abs(log((OS)/100))/(sqrt(pi^2+(log(OS/100))^2));    % Damping Ratio
Ts = .75;           % Settling Time
wn = 4/zeta/Ts;   % Natural Frequency
%% position based
s = tf('s');
int = 1/s;

%% Define Plant and Kp
plant = tf(Ka*Km*N/r,[M+J*(N^2)/(r^2) B/(r^2)]);
Kp = (2*zeta*wn*(r*(M+J*((N^2)/(r^2))))-B/r)/(Ka*Km*N);        % Proportional Gain from PI
Ki = wn^2*(r*(M+J*((N^2)/(r^2))))/Ka/Km/N;          % Integral Gain

%% Human Input
input_mode = input('input mode = ');
Tss = 2;
Fss = 2;
impulse_duration = 0;
if input_mode == 2
    impulse_duration = input('Duration of the impulse = ');
end
override_flag = 1;
%% Design Parameters
testcase = [0.6 0.6 3 3 100 input('Desired Virtual Mass = '); 60 0 60 0 0 input('Desired Virtual Elasticity = ');
    70 70 100 100 70 input('Desired Virtual Damping = '); 5 5 10 10 20 input('Desired Input Force = ')];
target_speed = 5;
%% Simulink
imax = 0;
vmax = 0;
tmax = 0;
rmax = 0;
figure(4),hold on
open('Impedance_Controller_Week5')
for i = 1:length(testcase(1,:))
    if i == 3
        i = i+1;
    end
    if i ==4
        i = i+1;
    end
    M_desired = testcase(1,i);
    K_desired = testcase(2,i);
    B_desired = testcase(3,i);
    Fss = testcase(4,i);
    sim('Impedance_Controller_Week5')
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(3,3,1),hold on
    plot(mot_spd),plot(ref_spd),title('system speed')
    legend('actual speed','reference speed')
    subplot(3,3,2),hold on
    plot(mot_pos),plot(ref_pos),title('system position')
    legend('actual position','reference position')
    subplot(3,3,3)
    plot(mot_acc)
    hold on
    plot(ref_acc), legend('motor acceleration','reference acceleration'), 
    title('motor acceleration')
    hold off
    subplot(3,3,4)
    plot(mot_fn), title('motor net force')
    subplot(3,3,5)
    plot(mot_f), title('motor force output')
    subplot(3,3,6)
    plot(toq_req), title('motor torque')
    subplot(3,3,7)
    plot(mot_v), title('motor voltage')
    subplot(3,3,8)
    plot(cur_req), title('motor current')
    subplot(3,3,9)
    plot(mot_rpm), title('motor RPM')
    suptitle(['M = ' num2str(M_desired) '(kg) K = '...
        num2str(K_desired) '(N/m) B = ' num2str(B_desired)...
        '(N/(m/s)) F = ' num2str(Fss) 'N'])
    figure(4)
    plot(mot_rpm.data,toq_req.data,'.')
    if (max(cur_req)>imax)
        imax = max(cur_req);
    end
    if (max(mot_v)>vmax&&i~=4)
        vmax = max(mot_v);
    end
    if (max(toq_req)>tmax)
        tmax = max(toq_req);
    end
    if (max(mot_rpm)>rmax&&i~=1)
        rmax = max(mot_rpm);
    end
end
imax
tmax
vmax
rmax