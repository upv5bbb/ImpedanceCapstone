% Richard Chen, Nathan Dabling and Fangzhong Guo 
% Capstone Design
% Week 5 Spr

clear all
close all
clc

%% System Requirements
M =  0.496+0.098+0.0045+0.003;          % Mass
OS = 5;        % Percent Overshoot
B = .00;       % Bearing Damping
%J = 7.06e-6;       % Wheel and Motor inertia
Km = .058;       % Motor Constant (Taken from ME 471 DC Motor Lab)
Ka = .4;       % Current Amp. Gain (Taken from ME 471 DC Motor Lab)
r = .01834/2;        % Wheel Radius
c = 2*pi*r;
N = 230/39;        % Gear Ratio
vol_limit = 24;
Rmot = 4;
zeta = abs(log((OS)/100))/(sqrt(pi^2+(log(OS/100))^2));    % Damping Ratio
Ts = 0.15;           % Settling Time
wn = 4/zeta/Ts;   % Natural Frequency
%% Define Plant and Kp
KF = 11.7102; % Actual K value from experiment
KK = KF*r/N; % Km and Ka
J= (6.339773548-M)*r^2/N^2; % Actual moment of inertia from experiment

% plant = tf(Ka*Km*N/r,[M+J*(N^2)/(r^2) B/(r^2)]);
plant = tf(KF,[M+J*(N^2)/(r^2) B/(r^2)]);
% Kp = (2*zeta*wn*(r*(M+J*((N^2)/(r^2))))-B/r)/(Ka*Km*N);        % Proportional Gain from PI
Kp = (2*zeta*wn*(r*(M+J*((N^2)/(r^2))))-B/r)/(KK*N);
% Ki = wn^2*(r*(M+J*((N^2)/(r^2))))/Ka/Km/N;          % Integral Gain
Ki = wn^2*(r*(M+J*((N^2)/(r^2))))/Ka/(KK/Ka)/N;          % Integral Gain

%% Human Input
simulation = input('run simulation? yes = 1, no =0  ');
impulse_duration = 0;
    target_speed = 5;
    override_flag = 1;
if simulation
    input_mode = input('input mode = ');
    Tss = 2;
    Fss = 20;
    Fs = timeseries(1,0);
    if input_mode == 2
        impulse_duration = input('Duration of the impulse = ');
    end
 %% Design Parameters
    testcase = [0.6 0.6 3 3 100 input('Desired Virtual Mass = '); 60 0 60 0 0 input('Desired Virtual Elasticity = ');
        70 70 100 100 70 input('Desired Virtual Damping = '); 5 5 10 10 20 input('Desired Input Force = ')];
    %% Simulink
    imax = 0;
    vmax = 0;
    tmax = 0;
    rmax = 0;
    figure(4),hold on
    title('torque map'),xlabel('rpm'),ylabel('torque(N*m)')
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
        legend('actual speed','reference speed'),ylabel('Speed(m/s)')
        subplot(3,3,2),hold on
        plot(mot_pos),plot(ref_pos),title('system position')
        legend('actual position','reference position'),ylabel('Position(m)')
        subplot(3,3,3),hold on
        plot(mot_acc),plot(ref_acc),title('motor acceleration') 
        legend('motor acceleration','reference acceleration')
        ylabel('Acceleration(m/s^2)')
        subplot(3,3,4)
        plot(mot_fn), title('motor net force'),ylabel('Force(N)')
        subplot(3,3,5)
        plot(mot_f), title('motor force output'),ylabel('Force(N)')
        subplot(3,3,6)
        plot(toq_req), title('motor torque'),ylabel('Torque(N*m)')
        subplot(3,3,7)
        plot(mot_v), title('motor voltage'),ylabel('Voltage(V)')
        subplot(3,3,8)
        plot(cur_req), title('motor current'),ylabel('Current(A)')
        subplot(3,3,9)
        plot(mot_rpm), title('motor RPM'),ylabel('RPM')
        suptitle(['M = ' num2str(M_desired) '(kg) K = '...
            num2str(K_desired) '(N/m) B = ' num2str(B_desired)...
            '(N/(m/s)) F = ' num2str(Fss) 'N'])
        figure(4)
        plot(abs(mot_rpm.data),abs(toq_req.data),'.')
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
end
%% Discrete Control
%---
PrintToFile = 1; % Set to 0 to print header file in Command Window only

HeaderFileName = ...
'C:\Users\Jeff\workspace\ImpedanceCapstone\ControllerHeader.h';
%-----continuous:
fs=200;        %---Sampling frequency for discrete filter
T=1/fs;         %---Sampling period
s = tf('s');
PI = Kp + Ki/s;
sPI = series(plant,PI);
%-----discrete equivalent:
plantd=c2d(plant,T,'tustin');
PId=c2d(PI,T,'tustin');
sPId=series(plantd,PId);

%---Biquad Cascade
% SOS is an L by 6 matrix with the following structure:
%         SOS = [ b01 b11 b21  1 a11 a21  
%                 b02 b12 b22  1 a12 a22
%                 ...
%                 b0L b1L b2L  1 a1L a2L ]
[b,a]=tfdata(PId,'v');       %---get discrete system coefficients
[sos,gain]=tf2sos(b,a);     %---convert to biquads
[ns,n]=size(sos);
for j=1:3                   %---Apply the gain to the final biquad
    sos(ns,j)=gain*sos(ns,j);
end

if PrintToFile
    fid=fopen(HeaderFileName,'W');    
else
    fid=1;   
end

%---Structure for cascade
comment=['PI controller'];
PrintToBiquadFHeaderFile(fid, sos, 'myFilter', T, comment);

if fid~=1
    fclose(fid);
end 
return
