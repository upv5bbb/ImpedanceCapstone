    close all; 
    %% System   
    M = 1;
    Fss = 5;
    M_desired = 0.6;
    K_desired = 60;
B_desired = 60;
%% Simulation
sim('Impedance_Controller_new')
 figure('units','normalized','outerposition',[0 0 1 1])
    subplot(3,3,1),hold on
    plot(mot_spd),plot(ref_spd),title('system speed')
    legend('actual speed','reference speed'),ylabel('Speed(m/s)')
    subplot(3,3,2),hold on
    plot(mot_pos),plot(ref_pos),title('system position')
    legend('actual position','reference position')
    ylabel('Position(m)')
    subplot(3,3,3),hold on
    plot(mot_acc),plot(ref_acc) 
    legend('motor acceleration','reference acceleration'), 
    title('motor acceleration'),ylabel('Acceleration(m/s^2)')
    hold off
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
%% Presentation Plots
for i = 1:length(testcase(1,:))
    M_desired = testcase(1,i);
    K_desired = testcase(2,i);
    B_desired = testcase(3,i);
    sim('Impedance_Controller_new')
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(1,3,1),hold on
    plot(toq_req), title('motor torque'),ylabel('Torque(N*m)')
    subplot(1,3,2),hold on
    plot(mot_v), title('motor voltage'),ylabel('Voltage(V)')
    subplot(1,3,3),hold on
    plot(cur_req), title('motor current'),ylabel('Current(A)')
    suptitle(['M = ' num2str(M_desired) '(kg) K = '...
    num2str(K_desired) '(N/m) B = ' num2str(B_desired)...
    '(N/(m/s)) F = ' num2str(Fss) 'N'])
end
%% Torque Map
plot(mot_rpm.data,toq_req.data,'.')
%% P2 Plots
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(3,2,1),hold on
    plot(mot_spd),plot(ref_spd),title('system speed')
    legend('actual speed','reference speed'),ylabel('Speed(m/s)')
    subplot(3,2,2),hold on
    plot(mot_pos),plot(ref_pos),title('system position')
    legend('actual position','reference position'),ylabel('Position(m)')
    subplot(3,2,3)
    plot(toq_req), title('motor torque'),ylabel('Torque(N*m)')
    subplot(3,2,4)
    plot(mot_v), title('motor voltage'),ylabel('Voltage(V)')
    subplot(3,2,5)
    plot(cur_req), title('motor current'),ylabel('Current(A)')
    subplot(3,2,6)
    plot(mot_rpm), title('motor RPM'),ylabel('RPM')
    suptitle(['M = ' num2str(M_desired) '(kg) K = '...
        num2str(K_desired) '(N/m) B = ' num2str(B_desired)...
        '(N/(m/s)) F = ' num2str(Fss) 'N'])
%% Validation
input_mode = 1;
M_desired = Reference_System(1,1);
K_desired = Reference_System(2,1);
B_desired = Reference_System(3,1);
Fss = Force(1,1);
T = 0:0.005:(size(Velocity)-1)*0.005;
error = mean(Velocity-V_Reference);
Fs = timeseries(Force,T,'Name','Fs');
sim('Impedance_Controller_Week5',[0 (length(Velocity)-1)*0.005]);
figure();hold on
plot(T,Velocity)
plot(T,V_Reference)
plot(mot_spd)
plot(ref_spd)
legend('actual','ref_C','model','ref_S')
axis([0 (length(Velocity)-1)*0.005 min(V_Reference)-0.1 max(V_Reference)+0.1])
title(['M = ' num2str(M_desired) '(kg) K = '...
        num2str(K_desired) '(N/m) B = ' num2str(B_desired)...
        '(N/(m/s))'])

 xlabel('Time(s)');ylabel('Velocity(m/s)');
%% Datas

%% Validation
input_mode = 1;
M_desired = Reference_System(1,1);
K_desired = Reference_System(2,1);
B_desired = Reference_System(3,1);
Fss = 2.4627;
T = 0.005:0.005:(size(Velocity))*0.005;
error = mean(Velocity-V_Reference);
Fs = timeseries(Force,T);
Fs.Data(1) = 0.00001;
sim('Impedance_Controller_Week5',[0 (length(Velocity))*0.005]);
figure();hold on
plot(T,Velocity)
plot(T,V_Reference)
plot(mot_spd)
plot(ref_spd)
legend('actual','ref_C','model','ref_S')
axis([0 (length(Velocity)-1)*0.005 min(Velocity)-0.1 max(Velocity)+0.1])
title(['M = ' num2str(M_desired) '(kg) K = '...
        num2str(K_desired) '(N/m) B = ' num2str(B_desired)...
        '(N/(m/s))'])
xlabel('Time(s)');ylabel('Velocity(m/s)');
%% plot
plot(T,Velocity,'-.g'),hold on
input_mode = 0;
M_desired = Reference_System(1,1);
K_desired = Reference_System(2,1);
B_desired = Reference_System(3,1);
Fss = 2.9812;
T = 0.005:0.005:(size(Velocity))*0.005;
Fs = timeseries(Force,T);
Fs.Data(1) = 0.00001;
error = mean(Velocity-V_Reference);
sim('Impedance_Controller_Week5',[0 (length(Velocity))*0.005]);
plot(T,V_Reference,'-.b')
plot(ref_spd,'-.r')
title('Plant Reponse VS Reference(Constant M and K)');
axis([0 (length(Velocity)-1)*0.005 min(Velocity)-0.1 max(Velocity)+0.1])
xlabel('Time(s)');ylabel('Velocity(m/s)');