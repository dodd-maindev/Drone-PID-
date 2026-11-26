close all; clear; clc;

tsim = 10.0; step = 1e-3; t= 0.0;
tspan = 0.0: step: tsim;
numdata = size(tspan, 2);

%- model parameter
mi = 1.7;
g = 9.8;
li = 0.2;

Ixi = 4e-3;
Iyi = 4e-3;
Izi = 8.4e-3;
Ii = diag([Ixi, Iyi, Izi]);

Jxi = 5.67e-4;
Jyi = 5.67e-4;
Jzi = 5.67e-4;
Ji = diag([Jxi, Jyi, Jzi]);

%- formation form
f1 = [4.0, 0.0, 0.0]';
f2 = [0.0, 4.0, 0.0]';
f3 = [-4.0, 0.0, 0.0]';
f4 = [0.0, -4.0, 0.0]';

f_i = cell(4,1);

f_i{1} = f1;
f_i{2} = f2;
f_i{3} = f3;
f_i{4} = f4;

w21 = 1;
w32 = 1;
w43 = 1;
w14 = 1;

b1 = 1;
b2 = 1;
b3 = 1;
b4 = 1;

%- state vector
x_i = cell(4, numdata);
y_i = cell(4, numdata);
z_i = cell(4, numdata);

dx_i = cell(4, numdata);
dy_i = cell(4, numdata);
dz_i = cell(4, numdata);

ddx_i = cell(4, numdata);
ddy_i = cell(4, numdata);
ddz_i = cell(4, numdata);

phi_i = cell(4, numdata);
theta_i = cell(4, numdata);
psi_i = cell(4, numdata);

dphi_i = cell(4, numdata);
dtheta_i = cell(4, numdata);
dpsi_i = cell(4, numdata);

ddphi_i = cell(4, numdata);
ddtheta_i = cell(4, numdata);
ddpsi_i = cell(4, numdata);

ux_i = cell(4, numdata);
uy_i = cell(4, numdata);
uz_i = cell(4, numdata);

tau_phi_i = cell(4, numdata);
tau_theta_i = cell(4, numdata);
tau_psi_i = cell(4, numdata);

p_0 = cell(1, numdata);
dp_0 = cell(1, numdata);

p_i = cell(4, numdata);
dp_i = cell(4, numdata);
ddp_i = cell(4, numdata);

p_des_i = cell(4, numdata);
dp_des_i = cell(4, numdata);
ddp_des_i = cell(4, numdata);

eta_i = cell(4, numdata);
deta_i = cell(4, numdata);
ddeta_i = cell(4, numdata);

eta_ref_i = cell(4, numdata);

eta_des_i = cell(4, numdata);
deta_des_i = cell(4, numdata);
ddeta_des_i = cell(4, numdata);

ep1_i = cell(4, numdata);
ep2_i = cell(4, numdata);
sp_i = cell(4, numdata);

e_eta_1i = cell(4, numdata);
e_eta_2i = cell(4, numdata);
s_eta_i = cell(4, numdata);

up_i = cell(4, numdata);
tau_i = cell(4, numdata);

u_i = cell(4, numdata);

%- initial condition
p_i{1,1} = [3.0, 2.0, 1.5]';
p_i{2,1} = [-2.0, 3.0, 1.0]';
p_i{3,1} = [-3.0, -2.0, 1.5]';
p_i{4,1} = [2.0, -3.0, 1.0]';

dp_i{1,1} = zeros(3,1);
dp_i{2,1} = zeros(3,1);
dp_i{3,1} = zeros(3,1);
dp_i{4,1} = zeros(3,1);

eta_i{1,1} = zeros(3,1);
eta_i{2,1} = zeros(3,1);
eta_i{3,1} = zeros(3,1);
eta_i{4,1} = zeros(3,1);

deta_i{1,1} = zeros(3,1);
deta_i{2,1} = zeros(3,1);
deta_i{3,1} = zeros(3,1);
deta_i{4,1} = zeros(3,1);

p_des_i{1,1} = zeros(3,1);
p_des_i{2,1} = zeros(3,1);
p_des_i{3,1} = zeros(3,1);
p_des_i{4,1} = zeros(3,1);

dp_des_i{1,1} = zeros(3,1);
dp_des_i{2,1} = zeros(3,1);
dp_des_i{3,1} = zeros(3,1);
dp_des_i{4,1} = zeros(3,1);

eta_des_i{1,1} = zeros(3,1);
eta_des_i{2,1} = zeros(3,1);
eta_des_i{3,1} = zeros(3,1);
eta_des_i{4,1} = zeros(3,1);

deta_des_i{1,1} = zeros(3,1);
deta_des_i{2,1} = zeros(3,1);
deta_des_i{3,1} = zeros(3,1);
deta_des_i{4,1} = zeros(3,1);

ddeta_des_i{1,1} = zeros(3,1);
ddeta_des_i{2,1} = zeros(3,1);
ddeta_des_i{3,1} = zeros(3,1);
ddeta_des_i{4,1} = zeros(3,1);

%- parameter of controller

k1i = 3;
k2i = 3;

alpha_p = 2;
beta_p = 0.5;
kpi1 = 2;
kpi2 = 0.1;

c1 = 3;
c2 = 3;
c3 = 1;
epsilon = 0.01;

alpha_e = 5;
beta_e = 0.5;
kei1 = 5;
kei2 = 1;


for i = 1:numdata
    disp(i);

    %- leader trajectory
    p_0{i} = [sin(0.15*t), cos(0.15*t), 3.0]';
    dp_0{i} = [0.15*cos(0.15*t), -0.15*sin(0.15*t), 0.0]';

    dp_des_1 = dp_des_i{1,i}- k1i*(w21*(p_des_i{1,i}- f1- p_des_i{2,i}+ f2)+ w14*(p_des_i{1,i}- f1- p_des_i{4,i}+ f4)+ b1*(p_des_i{1,i}- f1- p_0{i}));
    dp_des_2 = dp_des_i{2,i}- k1i*(w32*(p_des_i{2,i}- f2- p_des_i{3,i}+ f3)+ w21*(p_des_i{2,i}- f2- p_des_i{1,i}+ f1)+ b2*(p_des_i{2,i}- f2- p_0{i}));
    dp_des_3 = dp_des_i{3,i}- k1i*(w43*(p_des_i{3,i}- f3- p_des_i{4,i}+ f4)+ w32*(p_des_i{3,i}- f3- p_des_i{2,i}+ f2)+ b3*(p_des_i{3,i}- f3- p_0{i}));
    dp_des_4 = dp_des_i{4,i}- k1i*(w14*(p_des_i{4,i}- f4- p_des_i{1,i}+ f1)+ w43*(p_des_i{4,i}- f4- p_des_i{3,i}+ f3)+ b4*(p_des_i{4,i}- f4- p_0{i}));

    ddp_des_1 = - k2i*(w21*(dp_des_i{1,i}- dp_des_i{2,i})+ w14*(dp_des_i{1,i}- dp_des_i{4,i})+ b1*(dp_des_i{1,i}- dp_0{i}));
    ddp_des_2 = - k2i*(w32*(dp_des_i{2,i}- dp_des_i{3,i})+ w21*(dp_des_i{2,i}- dp_des_i{1,i})+ b2*(dp_des_i{2,i}- dp_0{i}));
    ddp_des_3 = - k2i*(w43*(dp_des_i{3,i}- dp_des_i{4,i})+ w32*(dp_des_i{3,i}- dp_des_i{2,i})+ b3*(dp_des_i{3,i}- dp_0{i}));
    ddp_des_4 = - k2i*(w14*(dp_des_i{4,i}- dp_des_i{1,i})+ w43*(dp_des_i{4,i}- dp_des_i{3,i})+ b4*(dp_des_i{4,i}- dp_0{i}));

    ddp_des_i{1,i} = ddp_des_1;
    ddp_des_i{2,i} = ddp_des_2;
    ddp_des_i{3,i} = ddp_des_3;
    ddp_des_i{4,i} = ddp_des_4;

    for k = 1:4
        %- redefine states
        x_i{k,i} = p_i{k,i}(1);
        y_i{k,i} = p_i{k,i}(2);
        z_i{k,i} = p_i{k,i}(3);

        dx_i{k,i} = dp_i{k,i}(1);
        dy_i{k,i} = dp_i{k,i}(2);
        dz_i{k,i} = dp_i{k,i}(3);

        phi_i{k,i} = eta_i{k,i}(1);
        theta_i{k,i} = eta_i{k,i}(2);
        psi_i{k,i} = eta_i{k,i}(3);

        dphi_i{k,i} = deta_i{k,i}(1);
        dtheta_i{k,i} = deta_i{k,i}(2);
        dpsi_i{k,i} = deta_i{k,i}(3);

        %- controller
        ep1_i{k,i} = p_i{k,i} - p_des_i{k,i};
        ep2_i{k,i} = dp_i{k,i} - dp_des_i{k,i};

        %sp_i{k,i} = ep2_i{k,i}+ alpha_p*abs(ep1_i{k,i}).^beta_p;
        sp_i{k,i} = ep2_i{k,i}+ alpha_p*ep1_i{k,i};

        %up_i{k,i} = Ii*dp_i{k,i}+ [0,0, mi*g]' - mi*(-ddp_des_i{k,i}+ alpha_p*beta_p*abs(ep1_i{k,i}).^(beta_p-1).*ep2_i{k,i}+ kpi1*sp_i{k,i}+ kpi2*tanh(sp_i{k,i}));
        up_i{k,i} = Ii*dp_i{k,i}+ [0,0, mi*g]' - mi*(-ddp_des_i{k,i}+ alpha_p*ep2_i{k,i}+ kpi1*sp_i{k,i}+ kpi2*tanh(sp_i{k,i}));
        
        uxi = up_i{k,i}(1);
        uyi = up_i{k,i}(2);
        uzi = up_i{k,i}(3);

        uTi = norm(up_i{k,i});
        psi_ref_i = 0.0;
        phi_ref_i = asin((uxi*sin(psi_ref_i)- uyi*cos(psi_ref_i))/uTi);
        theta_ref_i = atan((uxi*sin(psi_ref_i)+ uyi*cos(psi_ref_i))/uzi);

        eta_ref_i{k,i} = [phi_ref_i, theta_ref_i, psi_ref_i]';

        if i == 1
            eta_des_i{k,i} = eta_ref_i{k,i};
        end

        [time, data] = ode45(@hgo_3_dynamics, [t, t+ step], [eta_ref_i{k,i}', eta_des_i{k,i}', deta_des_i{k,i}', ddeta_des_i{k,i}']');

        eta_des_i{k,i} = data(1,4:6)';
        deta_des_i{k,i} = data(1,7:9)';
        ddeta_des_i{k,i} = data(1, 10:12)';

        eta_des_i{k,i+1} = data(end,4:6)';
        deta_des_i{k,i+1} = data(end,7:9)';
        ddeta_des_i{k,i+1} = data(end, 10:12)';

        e_eta_1i{k,i} = eta_i{k,i} - eta_des_i{k,i};
        e_eta_2i{k,i} = deta_i{k,i} - deta_des_i{k,i};

        s_eta_i{k,i} = e_eta_2i{k,i}+ alpha_e*e_eta_1i{k,i};

        f_phi_i = -(Jyi- Jzi)/Jxi*dtheta_i{k,i}*dpsi_i{k,i};
        f_theta_i = -(Jxi- Jzi)/Jyi*dphi_i{k,i}*dpsi_i{k,i};
        f_psi_i = -(Jyi- Jxi)/Jzi*dphi_i{k,i}*dtheta_i{k,i};

        F_i = [f_phi_i, f_theta_i, f_psi_i]';

        tau_i{k,i} = - Ji*(F_i -ddeta_des_i{k,i}+ alpha_e*e_eta_2i{k,i}+ kei1*s_eta_i{k,i}+ kpi2*tanh(s_eta_i{k,i}));

        tau_phi_i{k,i} = tau_i{k,i}(1);
        tau_theta_i{k,i} = tau_i{k,i}(2);
        tau_psi_i{k,i} = tau_i{k,i}(3);

        %- dynamic model
        upxi = uTi*(cos(phi_i{k,i})*sin(theta_i{k,i})*cos(psi_i{k,i}) + sin(phi_i{k,i})*sin(psi_i{k,i}));
        upyi = uTi*(cos(phi_i{k,i})*sin(theta_i{k,i})*sin(psi_i{k,i}) - sin(phi_i{k,i})*cos(psi_i{k,i}));
        upzi = uTi*cos(phi_i{k,i})*cos(theta_i{k,i});

        ddx_i{k,i} = -Ixi/mi*dx_i{k,i} + 1/mi*upxi;
        ddy_i{k,i} = -Iyi/mi*dy_i{k,i} + 1/mi*upyi;
        ddz_i{k,i} = -Izi/mi*dz_i{k,i} + 1/mi*upzi - g;

        ddphi_i{k,i} = -(Jyi- Jzi)/Jxi*dtheta_i{k,i}*dpsi_i{k,i}+ 1/Jxi*tau_phi_i{k,i};
        ddtheta_i{k,i} = -(Jxi- Jzi)/Jyi*dphi_i{k,i}*dpsi_i{k,i}+ 1/Jyi*tau_theta_i{k,i};
        ddpsi_i{k,i} = -(Jyi- Jxi)/Jzi*dphi_i{k,i}*dtheta_i{k,i}+ 1/Jzi*tau_psi_i{k,i};
    end

    %- update state
    for k = 1: 4
        if i == numdata
            break
        end

        t = t+ step;

        x_i{k,i+1} = x_i{k,i}+ dx_i{k,i}*step;
        y_i{k,i+1} = y_i{k,i}+ dy_i{k,i}*step;
        z_i{k,i+1} = z_i{k,i}+ dz_i{k,i}*step;

        phi_i{k,i+1} = phi_i{k,i}+ dphi_i{k,i}*step;
        theta_i{k,i+1} = theta_i{k,i}+ dtheta_i{k,i}*step;
        psi_i{k,i+1} = psi_i{k,i}+ dpsi_i{k,i}*step;

        phi_i{k,i+1} = min(max(phi_i{k,i+1}, -pi/3), pi/3);
        theta_i{k,i+1} = min(max(theta_i{k,i+1}, -pi/3), pi/3);
        psi_i{k,i+1} = min(max(psi_i{k,i+1}, -pi/3), pi/3);

        dx_i{k,i+1} = dx_i{k,i}+ ddx_i{k,i}*step;
        dy_i{k,i+1} = dy_i{k,i}+ ddy_i{k,i}*step;
        dz_i{k,i+1} = dz_i{k,i}+ ddz_i{k,i}*step;

        dphi_i{k,i+1} = dphi_i{k,i}+ ddphi_i{k,i}*step;
        dtheta_i{k,i+1} = dtheta_i{k,i}+ ddtheta_i{k,i}*step;
        dpsi_i{k,i+1} = dpsi_i{k,i}+ ddpsi_i{k,i}*step;

        p_i{k,i+1} = [x_i{k,i+1}, y_i{k,i+1}, z_i{k,i+1}]';
        dp_i{k,i+1} = [dx_i{k,i+1}, dy_i{k,i+1}, dz_i{k,i+1}]';
        ddp_i{k,i+1} = [ddx_i{k,i+1}, ddy_i{k,i+1}, ddz_i{k,i+1}]';

        eta_i{k,i+1} = [phi_i{k,i+1}, theta_i{k,i+1}, psi_i{k,i+1}]';
        deta_i{k,i+1} = [dphi_i{k,i+1}, dtheta_i{k,i+1}, dpsi_i{k,i+1}]';
        ddeta_i{k,i+1} = [ddphi_i{k,i+1}, ddtheta_i{k,i+1}, ddpsi_i{k,i+1}]';
    end

    dp_des_i{1,i+1} = dp_des_i{1,i}+ ddp_des_1*step;
    dp_des_i{2,i+1} = dp_des_i{2,i}+ ddp_des_2*step;
    dp_des_i{3,i+1} = dp_des_i{3,i}+ ddp_des_3*step;
    dp_des_i{4,i+1} = dp_des_i{4,i}+ ddp_des_4*step;

    p_des_i{1,i+1} = p_des_i{1,i}+ dp_des_1*step;
    p_des_i{2,i+1} = p_des_i{2,i}+ dp_des_2*step;
    p_des_i{3,i+1} = p_des_i{3,i}+ dp_des_3*step;
    p_des_i{4,i+1} = p_des_i{4,i}+ dp_des_4*step;
end

%- plot 3D 
data_p1 = zeros(3, numdata);
data_p2 = zeros(3, numdata);
data_p3 = zeros(3, numdata);
data_p4 = zeros(3, numdata);

for i = 1: numdata
    data_p1(:,i) = p_i{1,i};
    data_p2(:,i) = p_i{2,i};
    data_p3(:,i) = p_i{3,i};
    data_p4(:,i) = p_i{4,i};
end

figure(1)
plot3(data_p1(1,:), data_p1(2,:), data_p1(3,:)); hold on
plot3(data_p2(1,:), data_p2(2,:), data_p2(3,:)); hold on
plot3(data_p3(1,:), data_p3(2,:), data_p3(3,:)); hold on
plot3(data_p4(1,:), data_p4(2,:), data_p4(3,:)); grid on
xlabel('x (m)', 'Interpreter', 'latex');
ylabel('y (m)', 'Interpreter', 'latex');
zlabel('z (m)', 'Interpreter', 'latex');


