% Richard Chen, Nathan Dabling and Fangzhong Guo 
% Capstone Design
% Week 4 Spr
%   First, the PI is designed based on the reference input, then the
%   parameters are used to calculate the response from the human force
%   (treated like a disturbance) and the total response is determined
clear all
close all
clc

%% System Requirements
M = 1;          % Mass
OS = 10;        % Percent Overshoot
B = .00;       % Bearing Damping
J = .001;       % Wheel and Motor inertia
Km = .11;       % Motor Constant (Taken from ME 471 DC Motor Lab)
Ka = .79;       % Current Amp. Gain (Taken from ME 471 DC Motor Lab)
r = .0176/2;        % Wheel Radius
c = 2*pi*r;
Rmot = 4;
zeta = abs(log((OS)/100))/(sqrt(pi^2+(log(OS/100))^2));    % Damping Ratio
Ts = 2;           % Settling Time
wn = 4/zeta/Ts;   % Natural Frequency
%% position based
s = tf('s');
int = 1/s;

%% Define Plant and Kp
plant = tf(Ka*Km/r,[M+J/(r^2) B/(r^2)]);
Kp = (2*zeta*wn*(r*(M+J/(r^2)))-B/r)/(Ka*Km);        % Proportional Gain from PI

%% Target Point on Root Locus
x = -zeta*wn;       
y = wn*sqrt(1-zeta^2);
test_pt = complex(x,y); % Target point on Root Locus (Intersect of OS line 
                        % and Settling Time Line
%% Iterative Values
precision = .01;        % Determines precision of OS and test point 
increment = .01;        % Determines increment size in value adjustments
shift = .2;             % Determines proportional shift from test point. 
                        % If 'shift' is increased, OS increases and
                        % Settling Time decreases.
                        
f = 1;                  % Factor to adjust Ki until Root Locus approaches 
                        % intersecting test point (with shift)
                        
imPOLES = 0;            % First iteration comparison
OS_true = 100;           % Firt iteration comparison
iterOS = 0;             % Iteration counter for OS while loop

%% Iteration
% SCHEME:
% while -- Adjusts damping ratio until OS is met
%       if -- compares True Overshoot (from stepinfo()) to Target OS
%             increases damping ratio if OS too high
%       while -- Adjusts Ki to control location of zero on Root Locus
%             if -- compares imaginary component of the 2nd quadrant closed
%                   loop pole to that of the target point and adjust f
%                   accordingly
%       if -- breaks out of loop if damping ratio reaches undesired values

while OS_true >= (1+precision)*OS
    if OS_true >= (1+precision)*OS
        zeta = zeta+increment;
    end
    
    iterPOLES = 0;      % Iteration counter for debugging
    if shift ~= 0
        while imPOLES <= (shift-precision)*y || imPOLES >= (shift+precision)*y
            if imPOLES <= (shift-precision)*y
                f = f+increment;
            end
            if imPOLES >= (shift+precision)*y
                f = f-increment;
            end
            
            wn = 4/zeta/Ts;         % Recalculated Natural Frequency
            x = -zeta*wn;
            y = wn*sqrt(1-zeta^2);
            test_pt = complex(x,y); % Recalculated Test Point
            
            Ki = -f*x*Kp;           % Adjusted Ki
            
            C = zpk(-Ki/Kp,0,Kp);   % PI controller K
            G = C*plant;            % Open loop TF
            
            [K, poles] = rlocfind(G,test_pt);        % select target point
            R = rlocus(G,K);                         % get poles of locus
            imPOLES = imag(poles(1));
            iterPOLES = iterPOLES+1;
        end
    end
    if shift == 0
        while imPOLES <= (1-precision)*y || imPOLES >= (1+precision)*y
        if imPOLES <= (1-precision)*y
            f = f+increment;
        end
        if imPOLES >= (1+precision)*y
            f = f-increment;
        end
        
        wn = 4/zeta/Ts;         % Recalculated Natural Frequency
        x = -zeta*wn;
        y = wn*sqrt(1-zeta^2);
        test_pt = complex(x,y); % Recalculated Test Point
        
        Ki = -f*x*Kp;           % Adjusted Ki
        
        C = zpk(-Ki/Kp,0,Kp);   % PI controller K
        G = C*plant;            % Open loop TF
        
        [K, poles] = rlocfind(G,test_pt);        % select target point
        R = rlocus(G,K);                         % get poles of locus
        imPOLES = imag(poles(1));
        iterPOLES = iterPOLES+1;
        end
    end  
    G_closed = feedback(G*K,1);     % closed loop TF with PI
    G_info = stepinfo(G_closed);
    OS_true = G_info.Overshoot;     % True Overshoot
    Ts_true = G_info.SettlingTime;  % True Settling Time
    iterOS = iterOS+1;
    if zeta > .9 || zeta < 0
        break
    end
    if Ts_true > 1.5*Ts
        break
    end
end

Gf = tf(1,[M+J/(r^2) B/(r^2)]);
Gf_closed = feedback(Gf,Ka*Km/r*C);
[Yf,T]=step(Gf_closed);
[Yref]=step(G_closed,T);

%% Plotting
figure
hold on
rlocus(G)
sgrid(zeta, 5)                      % grid for damping ratio
plot([x,x],[-4,4],'k-')             % Settling Time Target Line
ylim([-4 4])
                                    % plot poles of locus
plot(R,'gs',    'MarkerEdgeColor','k',...           
                'MarkerFaceColor','g',...
                'MarkerSize',7) 
hold off

figure
step(G_closed,T)                      % Step Response of Closed Loop System
hold on
step(Gf_closed,T)
plot(T,Yref-Yf,'g-')
title('Step Response of Closed Loop System')
stepinfo(G_closed)
K
Ki
Kp
wn
OS_true
Ts_true
bandwidth(G_closed)
figure
pzmap(G_closed)
% figure
% bode(G_closed)
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
testcase = [0.2 0.2 0.2 1 1 1 100 100 100 input('Desired Virtual Mass = '); 60 0 60 60 60 0 0 0 60 input('Desired Virtual Elasticity = ');
    70 70 0 0 100 100 0 70 0 input('Desired Virtual Damping = '); 5 5 5 10 10 10 20 20 20 input('Desired Input Force = ')];
target_speed = 5;
%% Simulink
imax = 0;
vmax = 0;
tmax = 0;
rmax = 0;
figure(4),hold on
open('Impedance_Controller_new')
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
    sim('Impedance_Controller_new')
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