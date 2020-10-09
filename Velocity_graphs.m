%% Run each section seperately
% constants
a2 = 1;
d1 = 1;
%% Sec 1
% joints angles as a function of time
t = linspace(0,10,200);
q1_t = sin(t);
q2_t = cos(2*t);
d3_t = sin(3*t);
% joints angle velocities as a function of time
q1_dot=cos(t); 
q2_dot=-2*sin(2*t); 
d3_dot = 3*cos(3*t);
figure(1)
plot(t,q1_t)
title('Joints')
hold on
plot(t,q2_t)
plot(t,d3_t) 
legend('q1(t)','q2(t)','d3(t)')
hold off
figure(2)
plot(t,q1_dot)
title('Joint velocities')
hold on
plot(t,q2_dot)
plot(t,d3_dot) 
legend('q1_dot(t)','q2_dot(t)','d3_dot(t)')
hold off
%% sec 2
% prepare the vectors that will be plotted
J1_1 = zeros(1,200);
J1_2 = zeros(1,200);
J1_3 = zeros(1,200);

J2_1 = zeros(1,200);
J2_2 = zeros(1,200);
J2_3 = zeros(1,200);

J3_1 = zeros(1,200);
J3_2 = zeros(1,200);
J3_3 = zeros(1,200);

J4_1 = zeros(1,200);
J4_2 = zeros(1,200);
J4_3 = zeros(1,200);

J5_1 = zeros(1,200);
J5_2 = zeros(1,200);
J5_3 = zeros(1,200);

J6_1 = zeros(1,200);
J6_2 = zeros(1,200);
J6_3 = zeros(1,200);

X_dot = zeros(6,1,200);
%y_q1
% compute the forward kinematic matrix at each time step
for count = 1:200
    H = Rz(q1_t(count)) * Tz(d1) * Ry(q2_t(count)) * Tx(a2) * Tx(d3_t(count));
    % rotation matrix of the end effector
    R = H(1:3,1:3);
    % the inverse of the rotation matrix
    R_i = [R^-1 zeros(3,1);0 0 0 1];

    % get the jacobian of the first joint
    J1r = Rzd(q1_t(count)) * Tz(d1) * Ry(q2_t(count)) * Tx(a2) * Tx(d3_t(count)) * R_i;
    % get the jacobian elements
    J1 = [ J1r(1,4) J1r(2,4) J1r(3,4) J1r(3,2) J1r(1,3) J1r(2,1)].';

    %2nd joint
    J2r = Rz(q1_t(count)) * Tz(d1) * Ryd(q2_t(count)) * Tx(a2) * Tx(d3_t(count)) * R_i;
    % get the jacobian elements
    J2 = [ J2r(1,4) J2r(2,4) J2r(3,4) J2r(3,2) J2r(1,3) J2r(2,1)].';

    %3rd joint

    J3p = Rz(q1_t(count)) * Tz(d1) * Ry(q2_t(count)) * Tx(a2) * Txd(d3_t(count)) * R_i;
    % get the jacobian elements
    J3 = [ J3p(1,4) J3p(2,4) J3p(3,4) J3p(3,2) J3p(1,3) J3p(2,1)].';

    % Full jacobian
    J = [J1 J2 J3];
    X_dot(:,:,count) =  J * [ cos(t(count)); -2*sin(2*t(count)); 3*cos(3*t(count)) ];
    J1_1(count) = J(1,1);
    J1_2(count) = J(1,2);
    J1_3(count) = J(1,3);
    
    J2_1(count) = J(2,1);
    J2_2(count) = J(2,2);
    J2_3(count) = J(2,3);

    J3_1(count) = J(3,1);
    J3_2(count) = J(3,2);
    J3_3(count) = J(3,3);
    
    J4_1(count) = J(4,1);
    J4_2(count) = J(4,2);
    J4_3(count) = J(4,3);
    
    J5_1(count) = J(5,1);
    J5_2(count) = J(5,2);
    J5_3(count) = J(5,3);

    J6_1(count) = J(6,1);
    J6_2(count) = J(6,2);
    J6_3(count) = J(6,3);
end

%% plot each joint seperately
plt = tiledlayout(6,3); % Requires R2019b or later
nexttile
plot(t,J1_1,'r','DisplayName','x_q1(t)')
nexttile
plot(t,J1_2,'DisplayName','x_q2(t)')
nexttile
plot(t,J1_3,'g','DisplayName','x_d3(t)')
nexttile
plot(t,J2_1,'r','DisplayName','y_q1(t)')
nexttile
plot(t,J2_2,'DisplayName','y_q2(t)')
nexttile
plot(t,J2_3,'g','DisplayName','y_d3(t)')
nexttile
plot(t,J3_1,'r','DisplayName','z_q1(t)')
nexttile
plot(t,J3_2,'DisplayName','z_q2(t)')
nexttile
plot(t,J3_3,'g','DisplayName','z_d3(t)')
nexttile
plot(t,J4_1,'r','DisplayName','wx_q1(t)')
nexttile
plot(t,J4_2,'DisplayName','wx_q2(t)')
nexttile
plot(t,J4_3,'g','DisplayName','wx_d3(t)')
nexttile
plot(t,J5_1,'r','DisplayName','wy_q1(t)')
nexttile
plot(t,J5_2,'DisplayName','wy_q2(t)')
nexttile
plot(t,J5_3,'g','DisplayName','wy_d3(t)')
nexttile
plot(t,J6_1,'r','DisplayName','wz_q1(t)')
nexttile
plot(t,J6_2,'DisplayName','wz_q2(t)')
nexttile
plot(t,J6_3,'g','DisplayName','wz_d3(t)')
plt.Padding = 'none';
plt.TileSpacing = 'none';
legend('')
%% plotting of the toolframe velocities
x_dot = X_dot(1,:,:);
x_dot = x_dot(:);
y_dot = X_dot(2,:,:);
y_dot = y_dot(:);
z_dot = X_dot(3,1,:);
z_dot = z_dot(:);
wx_dot = X_dot(4,1,:);
wx_dot = wx_dot(:);
wy_dot = X_dot(5,1,:);
wy_dot = wy_dot(:);
wz_dot = X_dot(6,1,:);
wz_dot = wz_dot(:);
plt_2 = tiledlayout(6,1); % Requires R2019b or later
nexttile
plot(t,x_dot,'DisplayName','x_end(t)')
nexttile
plot(t,y_dot,'g','DisplayName','y_end(t)')
nexttile
plot(t,z_dot,'r','DisplayName','z_end(t)')
nexttile
plot(t,wx_dot,'DisplayName','wx_end(t)')
nexttile
plot(t,wy_dot,'g','DisplayName','wy_end(t)')
nexttile
plot(t,wz_dot,'r','DisplayName','wz_end(t)')
plt_2.Padding = 'none';
plt_2.TileSpacing = 'none';