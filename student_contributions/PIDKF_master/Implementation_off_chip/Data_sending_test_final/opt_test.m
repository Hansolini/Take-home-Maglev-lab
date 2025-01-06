%% 
parameters_maggy_V2;                %Load parameters and matrices
g = [
params.magnet.m*eye(3), zeros(3);
zeros(3), diag(params.magnet.I)
];
h = [zeros(6,1)]-[zeros(2,1);params.physical.g;zeros(3,1)];

N = 1000;           %How many runs per test

t_send_mem = zeros(1,N);        %Allocate memory
t_controller_mem = zeros(1,N);
t_force_mem = zeros(1,N);
t_receive_mem = zeros(1,N);
t_send_formatting_mem = zeros(1,N);
t_receive_formatting_mem = zeros(1,N);

arduinoRec = serialport("COM6",600000000);          %Open ports
arduinoSend = serialport("COM7",600000000);
send_data(arduinoSend,"0,0,0,0");           %Initial input
for i = 1:N
    tic;                %Receive
    x = receive_data(arduinoRec);
    t_receive_mem(i)=toc;

    tic;                %Compute forces and torque
    [F_m,F_s,tau_m,tau_s] = computeForceAndTorque(x',0,params,'fast');
    t_force_mem(i)=toc;

    tic                 %Calculate input
    u = [F_s;tau_s(1:2,:)]\(h(1:5)-[F_m;tau_m(1:2,:)]*I_m-g(1:5,1:6)*K_xyz*x');
    t_controller_mem(i)=toc;
    
    tic;                %Format input
    u = round(u);
    u = min(255,u);
    u = max(-255,u);
    u_send = string(u(1))+","+string(u(2))+","+string(u(3))+","+string(u(4));
    t_send_formatting_mem(i)=toc;

    tic;                %Send input
    send_data(arduinoSend,u_send);
    t_send_mem(i) = toc;
end
%Close the ports
delete(arduinoRec);
delete(arduinoSend);

%% Mean calc
send_mean = mean(t_send_mem)
send_format_mean = mean(t_send_formatting_mem);
rec_format_mean = mean(t_receive_formatting_mem);
controller_mean= mean(t_controller_mem);
force_mean = mean(t_force_mem);
receive_mean = mean(t_receive_mem(2:end))

Arduino_total = csvread('PATH TO ARDUINO_IMP_TOTAL_COMP_TIME');
Arduino_calc = csvread('PATH TO ARDUINO_IMP_CALC_TIME');
Arduino_cont = csvread('PATH TO ARDUINO_IMP_CONTROLLER_TIME');

arduino_controller_mean = mean(Arduino_cont./1000000);
arduino_calculation_mean = mean(Arduino_calc./1000000);

%% Plotting (Original Matlab with transport times)
close all
figure
barh([0],[receive_mean force_mean controller_mean send_format_mean send_mean],'stacked')
set(gca,'YTickLabel',[]);
grid on
xlabel("$t$(s)",'Interpreter','latex')
title("Composition of running time of entire controller w/ data transport between Matlab and Arduino, 1ms sampling",'Interpreter','latex')
legend("Receiving time","Force and torque calculation","Input calculation","Send formatting","Send time")
txt = '\leftarrow sin(\pi) = 0';
annotation('textbox', [0, 0.6, 0, 0], 'string', 'Total comp. time')

%% Plotting (Original Matlab vs pure Arduino)
close all
figure
barh([force_mean controller_mean; arduino_calculation_mean arduino_controller_mean],'stacked')
set(gca,'YTickLabel',[]);
grid on
xlabel("$t$(s)",'Interpreter','latex')
title("Composition of running time of entire controller",'Interpreter','latex')
legend("Force and torque calculation","Input calculation")
txt = '\leftarrow sin(\pi) = 0';
annotation('textbox', [0, 0.7, 0, 0], 'string', 'Teensy implementation')
annotation('textbox', [0, 0.45, 0, 0], 'string', 'MATLAB implementation')


% figure
% barh([0],[arduino_calculation_mean arduino_controller_mean],'stacked')
% set(gca,'YTickLabel',[]);
% grid on
% xlabel("$t$(s)",'Interpreter','latex')
% title("Composition of running time of entire controller, Arduino implementation",'Interpreter','latex')
% legend("Force and torque calculation","Input calculation")
% txt = '\leftarrow sin(\pi) = 0';
% annotation('textbox', [0, 0.6, 0, 0], 'string', 'Total computational time')

%% Direct comparison with transport time for Matlab
close all
figure
barh([receive_mean force_mean controller_mean send_format_mean send_mean; ...
      0 arduino_calculation_mean arduino_controller_mean 0 0],'stacked')
set(gca,'YTickLabel',[]);
grid on
xlabel("$t$(s)",'Interpreter','latex')
title("Comparison of total running time for Teensy and Matlab implementations",'Interpreter','latex')
legend("Receiving time","Force and torque calculation","Input calculation","Send formatting","Send time")
txt = '\leftarrow sin(\pi) = 0';
annotation('textbox', [0, 0.5, 0, 0], 'string', 'Matlab implementation w/ transport delay')
annotation('textbox', [0, 0.7, 0, 0], 'string', 'Direct implementation on Teensy')

%% Functions

function send_data(obj,u)
    write(obj,u,"string");
end

function rec = receive_data(obj)
    rec = read(obj,10,"single");
end