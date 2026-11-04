close all; clear; clc;

freq_noise_p = 50*rand(3,10);
freq_noise_e = 50*rand(3,10);

%- time vector
tsim = 60.0; step = 1e-3; t = 0.0;
tspan = 0.0: step: tsim;
numdata = size(tspan, 2);

%- quadrotor parameter
m = 1.7;
g = 9.8;
J = diag([4.8, 4.8, 9.8])*1e-2;

%- stored data vector
data_p = zeros(3, numdata);
data_dp = zeros(3, numdata);
data_pd = zeros(3, numdata);
data_dpd = zeros(3, numdata);
data_Wp = zeros(9, numdata);
data_dis_p = zeros(3, numdata);
data_dis_p_hat = zeros(3, numdata);
data_eta_ref = zeros(3,numdata);
data_eta_d = zeros(3, numdata);
data_deta_d = zeros(3, numdata);
data_eta = zeros(3, numdata);
data_deta = zeros(3, numdata);
data_We = zeros(9, numdata);
data_dis_eta = zeros(3, numdata);
data_dis_eta_hat = zeros(3, numdata);

%- critic term
Qp = eye(6);
Rp = eye(3);
kp1 = 0.5;
kp2 = 0.2;

Qe = 10*eye(6);
Re = eye(3);
ke1 = 0.5;
ke2 = 0.2;

%- ESO parameter
c1 = 6;
c2 = 11;
c3 = 6;
epsilon = 0.01;

%- initizalization
%p = [-2.0, 0.0, 1.0]';
p = [2.0, 4.0, 1.5]';
dp = [0.0, 0.0, 0.0]';
p_hat = zeros(3,1);
dp_hat = zeros(3,1);
eta = zeros(3,1);
deta = zeros(3,1);
eta_hat = zeros(3,1);
deta_hat = zeros(3,1);
eta_d = zeros(3,1);
deta_d = zeros(3,1);
ddeta_d = zeros(3,1);
dis_p_hat = zeros(3,1);
dis_eta_hat = zeros(3,1);
Wp = ones(9,1);
We = ones(9,1);

%- for loop
for i = 1: numdata
    disp(i);

    %- position reference
    pd = [5*sin(0.2*t), 5*cos(0.2*t), t]';
    dpd = [cos(0.2*t), -sin(0.2*t), 1.0]';
    ddpd = [-0.2*sin(0.2*t), -0.2*cos(0.2*t), 0.0]';


    %- position controller
    ep = p - pd;
    dep = dp - dpd;
    zp = [ep', dep']';

    Fp = [dep', zeros(3,1)']';
    Gp = [zeros(3,3), 1/m*eye(3)]';

    up1 = m*g*[0,0,1]'+ m*ddpd;
    up2 = -1/2*Rp\Gp'*grad_af(zp)'*Wp;
    up = up1+ up2- m*dis_p_hat;

    %- update weight of position critic
    gamma_p = grad_af(zp)*(Fp+ Gp*up2);
    eB_p = Wp'*gamma_p+ zp'*Qp*zp+ up2'*Rp*up2;
 
    if gamma_p < 0
        lp = 0;
    else
        lp = 1;
    end
    varphi_p = 1/2*grad_af(zp)*Gp*inv(Rp)*Gp'*grad_af(zp)';
    dWp = -kp1*gamma_p*eB_p/(1+ gamma_p'*gamma_p)^2+ kp2*lp*varphi_p*Wp;

    %- extended state observer
    if i  == 1
        p_hat = p;
    end
    p_hat_1dot = dp_hat - c1/epsilon*(p_hat - p);
    p_hat_2dot = -g*[0,0,1]' + 1/m*B2E_mat(eta)*norm(up)*[0,0,1]' - c2/epsilon^2*(p_hat - p)+ dis_p_hat;
    d_dis_p_hat = -c3/epsilon^3*(p_hat- p);

    %- desired attitude
    psi_ref = 0.0;
    upx = up(1);
    upy = up(2);
    upz = up(3);
    uT = norm(up); 
    phi_ref = asin((upx*sin(psi_ref)- upy*cos(psi_ref))/uT);
    theta_ref = atan((upx*cos(psi_ref)+ upy*sin(psi_ref))/upz);

    eta_ref = [phi_ref, theta_ref, psi_ref]';

    %- high gain observer
    if i == 1
        eta_d = eta_ref;
    end
    
    eta_d_1dot = deta_d- c1/epsilon*(eta_d- eta_ref);
    eta_d_2dot = ddeta_d- c2/epsilon^2*(eta_d- eta_ref);
    eta_d_3dot = -c3/epsilon^3*(eta_d- eta_ref);

    %- computing
    e_eta = eta - eta_d;
    de_eta = deta - deta_d;
    z_eta = [e_eta', de_eta']';

    Fe = [de_eta', (J\Cor_mat(J,eta,deta)*de_eta)']';
    Ge = [zeros(3,3), inv(J)]';

    ue1 = -Cor_mat(J,eta,deta)*deta_d+ J*ddeta_d;
    ue2 = -1/2*Re\Ge'*grad_af(z_eta)'*We;
    ue = ue1+ ue2- J*dis_eta_hat;

    %- update weight of attitude critic
    gamma_e = grad_af(z_eta)*(Fe+ Ge*ue2);
    eB_e = We'*gamma_e+ z_eta'*Qe*z_eta+ ue2'*Re*ue2;
 
    if gamma_e < 0
        le = 0;
    else
        le = 1;
    end
    varphi_e = 1/2*grad_af(z_eta)*Ge*inv(Re)*Ge'*grad_af(z_eta)';
    dWe = -ke1*gamma_e*eB_e/(1+ gamma_e'*gamma_e)^2+ ke2*le*varphi_e*We;

     %- extended state observer
    if i  == 1
        eta_hat = eta;
    end
    
    eta_hat_1dot = deta_hat - c1/epsilon*(eta_hat- eta);
    eta_hat_2dot = Cor_mat(J,eta,deta)*deta+ J\ue - c2/epsilon^2*(eta_hat- eta)+ dis_eta_hat;
    d_dis_eta_hat = -c3/epsilon^3*(eta_hat- eta);

    %- quadrotor dynamicss0
    dis_p = 0.1*sum(sin(freq_noise_p*t), 2);
    dis_eta = 0.02*sum(sin(freq_noise_e*t), 2);

    dis_p = 0.10*[sin(1+ 0.7*t)*cos(0.5*t), sin(1.7+ 0.8*t)*cos(0.6*t), sin(0.5+ 0.4*t)*cos(0.8*t)]';
    dis_eta = 0.02*[sin(1.1+ 0.2*t)*cos(0.3*t), sin(0.4+ 0.3*t)*cos(0.1*t), sin(0.9+ 0.4*t)*cos(0.1*t)]';

    %dis_p = zeros(3,1);
    %dis_eta = zeros(3,1);


    p_1dot = dp;
    p_2dot = -g*[0,0,1]' + 1/m*B2E_mat(eta)*uT*[0,0,1]' + 1/m*dis_p;
    p_2dot = -g*[0,0,1]' + 1/m*up + 1/m*dis_p;
    eta_1dot = deta;
    eta_2dot = Cor_mat(J,eta,deta)*deta+ J\ue + J\dis_eta;

    %- store data
    data_p(:,i) = p;
    data_dp(:,i) = dp;
    data_pd(:,i) = pd;
    data_dpd(:,i) = dpd;
    data_Wp(:,i) = Wp;
    data_dis_p(:,i) = dis_p;
    data_dis_p_hat(:,i) = m*dis_p_hat;
    data_eta_ref(:,i) = eta_ref;
    data_eta_d(:,i) = eta_d;
    data_deta_d(:,i) = deta_d;
    data_eta(:,i) = eta;
    data_deta(:,i) = deta;
    data_We(:,i) = We;
    data_dis_eta(:,i) = dis_eta;
    data_dis_eta_hat(:,i) = J*dis_eta_hat;

    if i == numdata
        break;
    end

    t = t+ step;
    dp = dp+ p_2dot*step;
    p = p + p_1dot*step;
    Wp = Wp+ dWp*step;
    dp_hat = dp_hat+ p_hat_2dot*step;
    p_hat = p_hat+ p_hat_1dot*step;
    dis_p_hat = dis_p_hat+ d_dis_p_hat*step;
    ddeta_d = ddeta_d+ eta_d_3dot*step;
    deta_d = deta_d+ eta_d_2dot*step;
    eta_d = eta_d + eta_d_1dot*step;
    deta = deta+ eta_2dot*step;
    eta = eta+ eta_1dot*step;
    deta_hat = deta_hat+ eta_hat_2dot*step;
    eta_hat = eta_hat+ eta_hat_1dot*step;
    dis_eta_hat = dis_eta_hat+ d_dis_eta_hat*step;
    We = We+ dWe*step;
end


ex = data_p(1,:) - data_pd(1,:);
ey = data_p(2,:) - data_pd(2,:);
ez = data_p(3,:) - data_pd(3,:);

IAE_x = trapz(tspan, abs(ex))
IAE_y = trapz(tspan, abs(ey))
IAE_z = trapz(tspan, abs(ez))

ISE_x = trapz(tspan, ex.^2)
ISE_y = trapz(tspan, ey.^2)
ISE_z = trapz(tspan, ez.^2)

ITAE_x = trapz(tspan, tspan.*abs(ex))
ITAE_y = trapz(tspan, tspan.*abs(ey))
ITAE_z = trapz(tspan, tspan.*abs(ez))

ITSE_x = trapz(tspan, tspan.*ex.^2)
ITSE_y = trapz(tspan, tspan.*ey.^2)
ITSE_z = trapz(tspan, tspan.*ez.^2)




figure(5)
subplot(3,1,1)
hold on; plot(tspan, data_p(1,:) - data_pd(1,:), 'r-', 'LineWidth', 1.0);
subplot(3,1,2)
hold on; plot(tspan, data_p(2,:) - data_pd(2,:), 'r-', 'LineWidth', 1.0);
subplot(3,1,3)
hold on; plot(tspan, data_p(3,:) - data_pd(3,:), 'r-', 'LineWidth', 1.0);


figure(2)
subplot(3,1,1)
hold on; plot(tspan, data_eta(1,:)- data_eta_d(1,:), 'r-', 'LineWidth', 1.0);
subplot(3,1,2)
hold on; plot(tspan, data_eta(2,:)- data_eta_d(2,:), 'r-', 'LineWidth', 1.0);
subplot(3,1,3)
hold on; plot(tspan, data_deta(3,:)- data_eta_d(3,:), 'r-', 'LineWidth', 1.0);

figure(4)
plot3(data_pd(1,:), data_pd(2,:), data_pd(3,:), 'k-','LineWidth',1.0);
hold on; grid on; 
plot3(data_p(1,:), data_p(2,:), data_p(3,:), 'r--','LineWidth',1.0); 
xlabel('$x$ $[m]$','Interpreter','latex','FontSize',11);
ylabel('$y$ $[m]$','Interpreter','latex','FontSize',11); 
zlabel('$z$ $[m]$','Interpreter','latex','FontSize',11);


figure(13)
plot(tspan, data_Wp, 'LineWidth', 1.0);
ylabel('$W_p$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(a)  Thời gian [s]');

figure(14)
subplot(3,1,1)
plot(tspan, data_dis_p(1,:), 'k-', tspan, data_dis_p_hat(1,:), 'r--', 'LineWidth', 1.0);
ylabel('$d_x$ $[m/s^2]$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(a)  Thời gian [s]');
subplot(3,1,2)
plot(tspan, data_dis_p(2,:), 'k-', tspan, data_dis_p_hat(2,:), 'r--', 'LineWidth', 1.0);
ylabel('$d_y$ $[m/s^2]$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(b)  Thời gian [s]');
subplot(3,1,3)
plot(tspan, data_dis_p(3,:), 'k-', tspan, data_dis_p_hat(3,:), 'r--', 'LineWidth', 1.0);
ylabel('$d_z$ $[m/s^2]$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(c)  Thời gian [s]');


figure(9)
plot(tspan, data_We, 'LineWidth', 1.0);
ylabel('$W_e$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(a)  Thời gian [s]');

figure(10)
subplot(3,1,1)
plot(tspan, data_dis_eta(1,:), 'k-', tspan, data_dis_eta_hat(1,:), 'r--', 'LineWidth', 1.0);
ylabel('$d_\phi$ $[rad//s^2]$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(a)  Thời gian [s]');
subplot(3,1,2)
plot(tspan, data_dis_eta(2,:), 'k-', tspan, data_dis_eta_hat(2,:), 'r--', 'LineWidth', 1.0);
ylabel('$d_\theta$ $[rad/s^2]$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(b)  Thời gian [s]');
subplot(3,1,3)
plot(tspan, data_dis_eta(3,:), 'k-', tspan, data_dis_eta_hat(3,:), 'r--', 'LineWidth', 1.0);
ylabel('$d_\psi$ $[rad/s^2]$', 'Interpreter','latex', 'FontSize', 14);
xlabel('(c)  Thời gian [s]');





%% function

function output = af(input)
    e1 = input(1);
    e2 = input(2);
    e3 = input(3);

    de1 = input(4);
    de2 = input(5);
    de3 = input(6);

    output = [e1^2, e2^2, e3^2, de1^2, de2^2, de3^2, e1*de1, e2*de2, e3*de3]';
end

function output = grad_af(input)
    e1 = input(1);
    e2 = input(2);
    e3 = input(3);

    de1 = input(4);
    de2 = input(5);
    de3 = input(6);

    c1 = [2*e1, 0.0, 0.0, 0.0, 0.0, 0.0, de1, 0.0, 0.0]';
    c2 = [0.0, 2*e2, 0.0, 0.0, 0.0, 0.0, 0.0, de2, 0.0]';
    c3 = [0.0, 0.0, 2*e3, 0.0, 0.0, 0.0, 0.0, 0.0, de3]';
    c4 = [0.0, 0.0, 0.0, 2*de1, 0.0, 0.0, e1, 0.0, 0.0]';
    c5 = [0.0, 0.0, 0.0, 0.0, 2*de2, 0.0, 0.0, e2, 0.0]';
    c6 = [0.0, 0.0, 0.0, 0.0, 0.0, 2*de3, 0.0, 0.0, e3]';

    output = [c1, c2, c3, c4, c5, c6];
end

function result = B2E_mat(eta)
    phi = eta(1);
    theta = eta(2);
    psi = eta(3);
    
    b11 = cos(theta)*cos(psi);
    b12 = cos(psi)*sin(phi)*sin(theta)- cos(phi)*sin(psi);
    b13 = sin(phi)*sin(psi)+ cos(phi)*cos(psi)*sin(theta);

    b21 = cos(theta)*sin(psi);
    b22 = cos(phi)*cos(psi)+ sin(phi)*sin(theta)*sin(psi);
    b23 = cos(phi)*sin(theta)*sin(psi)- cos(psi)*sin(phi);

    b31 = -sin(theta);
    b32 = cos(theta)*sin(phi);
    b33 = cos(phi)*cos(theta);

    result = [b11, b12, b13;...
              b21, b22, b23;...
              b31, b32, b33];
end

function result = Cor_mat(J, eta, deta)
        Jx = J(1,1);
        Jy = J(2,2);
        Jz = J(3,3);

        phi = eta(1);
        theta = eta(2);
        psi = eta(3);

        dphi = deta(1);
        dtheta = deta(2);
        dpsi = deta(3);

        c11 = 0;
        c12 = (Jy- Jz)*(dtheta*cos(phi)*sin(phi) + dpsi*sin(phi)^2*cos(theta)) ...
                + (Jz- Jy)*dpsi*cos(phi)^2*cos(theta)...
                - Jx*dpsi*cos(theta);
        c13 = (Jz- Jy)*dpsi*cos(phi)*sin(phi)*cos(theta)^2;
        c21 = (Jz- Jy)*(dtheta*cos(phi)*sin(phi)+ dpsi*sin(phi)^2*cos(theta)) ...
                + ((Jy- Jz)*dpsi*cos(phi)^2*cos(theta) ...
                + Jx*dpsi*cos(theta));
        c22 = (Jz- Jy)*dphi*cos(phi)*sin(phi);
        c23 = -Jx*dpsi*sin(theta)*cos(theta) ...
                + Jz*dpsi*cos(phi)^2*cos(theta)*sin(theta) ...
                + Jy*dpsi*sin(phi)*cos(theta)*sin(theta);
        c31 = (Jy- Jz)*dpsi*cos(theta)^2*sin(phi)*cos(phi) ...
                - Jx*dtheta*cos(theta);
        c32 = (Jz- Jy)*(dtheta*cos(phi)*sin(phi)*sin(theta)+ dphi*sin(phi)^2*cos(theta)) ...
                +(Jy- Jz)*dphi*cos(phi)^2*cos(theta) ...
                + Jx*dpsi*sin(theta)*cos(theta) ...
                - Jy*dpsi*sin(phi)^2*sin(theta)*cos(theta) ...
                - dpsi*cos(phi)^2*cos(theta)*sin(theta);
        c33 = -Jz*dtheta*cos(phi)^2*sin(theta)*cos(theta) ...
                - Jy*dtheta*sin(phi)^2*cos(theta)*sin(theta) ...
                + (Jy- Jz)*dphi*cos(phi)*sin(phi)*cos(theta)^2 ...
                + Jx*dtheta*sin(theta)*cos(theta);

        result = J*[c11 c12 c13;...
                    c21 c22 c23;...
                    c31 c32 c33];
end


