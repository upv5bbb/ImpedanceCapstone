    close all; 
    %% System   
    M = 1;
    Fss = 5;
    M_desired = 0.6;
    K_desired = 10;
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