 clear; clc;
m = 1.7; kw = 1; kt = 1; g = 9.8; l = 0.2; Jx = 4.8e-3; Jy = 4.8e-3; Jz = 9.8e-3;
freq_noise_p = 50*rand(3,10);
freq_noise_e = 50*rand(3,10);

t = 0.0; step = 5e-3; tsim = 60.0;
tspan = 0.0: step: tsim;

numdata = size(tspan, 2);

data_xp = zeros(3, numdata);
data_xe = zeros(3, numdata);
data_xp_ref = zeros(3, numdata);
data_xe_ref = zeros(3, numdata);
data_dxe_ref = zeros(3, numdata);
data_up = zeros(3, numdata);
data_ctrl_input = zeros(4, numdata);

xp = [2, 4, 1.5]';
xe = [0.0, 0.0, 0.0]';
dxp = [0.0, 0.0, 0.0]';
dxe = [0.0, 0.0, 0.0]';

alpha_p = 0.5;
kp11 = diag([1.7, 1.7, 1.7]);
kp12 = diag([0.5, 0.5, 0.5]);
kp2 = diag([2, 2, 2]);
kp3 = diag([0.2, 0.2, 0.2]);

alpha_e = 0.5;
ke11 = diag([3.0, 3.0, 3.0]);
ke12 = diag([0.5, 0.5, 0.5]);
ke2 = diag([5.0, 5.0, 5.0]);
ke3 = diag([0.2, 0.2, 0.2]);

int_ep_sign = 0.0;
int_ee_sign = 0.0;
for i = 1:numdata

    xp_ref = [5*sin(0.2*t), 5*cos(0.2*t), t]';
    dxp_ref = [cos(0.2*t), -sin(0.2*t), 1.0]';
    ddxp_ref = [-0.2*sin(0.2*t), -0.2*cos(0.2*t), 0.0]';

    ep = xp- xp_ref;
    dep = dxp- dxp_ref;
    int_ep_sign = int_ep_sign+ abs(ep).^alpha_p.*sign(ep)*step;
    sp = dep+ kp11*ep+ kp12*int_ep_sign;
    up_eq = ddxp_ref- kp11*dep- kp12*abs(ep).^alpha_p.*sign(ep);
    up_sw = - kp2*sp- kp3*tanh(sp);
    up = m/kw*(up_eq+ up_sw+ g*e(3,3));

    data_xp(:,i) = xp;
    data_xp_ref(:,i) = xp_ref;
    data_up(:,i) = up;
    
    phi = xe(1); theta = xe(2); psi = xe(3);
    data_xe(:,i) = xe;

    uz = up(3)/cos(phi)/cos(theta);
    psi_ref = 0.0;
    phi_ref = asin((cos(phi)*sin(theta)*sin(psi)- up(2)/uz)/cos(psi));
    theta_ref = asin((up(1)/uz- sin(phi)*sin(psi))/cos(phi)/cos(psi));
    xe_ref = [phi_ref, theta_ref, psi_ref]';

    if t == 0
        dxe_ref = [0.0, 0.0, 0.0]';
        ddxe_ref = [0.0, 0.0, 0.0]';
    else
        dxe_ref = 1/step*(xe_ref - data_xe_ref(:,i-1));
        ddxe_ref = 1/step*(dxe_ref - data_dxe_ref(:, i-1));
    end

    data_xe_ref(:,i) = xe_ref;
    data_dxe_ref(:,i) = dxe_ref;

    C = rot_mat([Jx, Jy, Jz], [xe; dxe]);
    Be = diag([1/Jx*l*kw, 1/Jy*l*kw, 1/Jz*kt]);

    ee = xe - xe_ref;
    dee = dxe - dxe_ref;
    
    int_ee_sign = int_ee_sign+ abs(ee).^alpha_e.*sign(ee)*step;
    se = dee+ ke11*ee+ ke12*int_ee_sign;
    ue_eq = ddxe_ref- ke11*dee- ke12*abs(ee).^alpha_e.*sign(ee);
    ue_sw = - ke2*se- ke3*tanh(se);
    ue = Be\(ue_eq+ ue_sw- C*dxe);

    data_ctrl_input(:,i) = [uz; ue];
    timedelay = 0.05;
    if t < timedelay
        uz = 0.0;
        ue = zeros(3,1);
    else
        uz = data_ctrl_input(1, i- round(timedelay/step, 0)+2);
        ue = data_ctrl_input(2:4, i- round(timedelay/step, 0)+2);
    end

    noise_p = 0.1*sum(sin(freq_noise_p*t), 2);
    noise_e = 0.02*sum(sin(freq_noise_e*t), 2);

    %dis_p = 0.1*[sin(1+ 0.7*t)*cos(0.5*t), sin(1.7+ 0.8*t)*cos(0.6*t), sin(0.5+ 0.4*t)*cos(0.8*t)]';
    %dis_e = 0.02*[sin(1.1+ 0.2*t)*cos(0.3*t), sin(0.4+ 0.3*t)*cos(0.1*t), sin(0.9+ 0.4*t)*cos(0.1*t)]';

    %dis_p = zeros(3,1);
    %dis_e = zeros(3,1);

    dis_p = [0.2*sin(0.45*t)+ 0.05*cos(1.2*t); ...
                   0.25*sin(0.25*t)*cos(0.5*t)+ 0.1*cos(0.7*t); ...
                   0.15*sin(0.45*t+ 0.3)+ 0.25*cos(0.5*t)];

    dis_e = [0.02*sin(0.6*t)+ 0.01*cos(1.1*t); ...
                  0.018*sin(0.5*t+ 0.5)+ 0.015*cos(t); ...
                  0.01*sin(0.4*t)+ 0.03*cos(0.8*t)];

    ddxp = kw/m*uz*trans_mat(xe)*e(3,3) - g*e(3,3)+ dis_p;
    ddxe = C*dxe+ Be*ue+ dis_e;
   
    t = t+ step;
    
    dxp = dxp+ ddxp*step;
    dxe = dxe+ ddxe*step;
    xp = xp+ dxp*step;
    xe = xe+ dxe*step;
end

data_ep = data_xp- data_xp_ref;
data_ee = data_xe- data_xe_ref;

ex = data_ep(1,:);
ey = data_ep(2,:);
ez = data_ep(3,:);

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


figure(1)
subplot(3,1,1)
hold on; grid on;
plot(tspan, data_xp_ref(1,:), 'k-',tspan, data_xp(1,:), 'r--', 'LineWidth',1); 
ylabel('$x$ $[m]$','Interpreter','latex','FontSize',11)
xlabel('(a) Thời gian [s]')
subplot(3,1,2)
hold on; grid on;
plot(tspan, data_xp_ref(2,:), 'k-',tspan, data_xp(2,:), 'r--', 'LineWidth',1); 
ylabel('$y$ $[m]$','Interpreter','latex','FontSize',11)
xlabel('(b) Thời gian [s]')
subplot(3,1,3)
hold on; grid on;
plot(tspan, data_xp_ref(3,:), 'k-',tspan, data_xp(3,:), 'r--', 'LineWidth',1);
ylabel('$z$ $[m]$','Interpreter','latex','FontSize',11); 
xlabel('(c) Thời gian [s]')


figure(2)
subplot(3,1,1)
hold on; grid on;
plot(tspan, -data_xe_ref(1,:)+ data_xe(1,:), 'r-', 'LineWidth',1); 
ylabel('$e_\phi$ $[rad]$','Interpreter','latex','FontSize',11)
xlabel('(a) Thời gian [s]')
subplot(3,1,2)
hold on; grid on;
plot(tspan, -data_xe_ref(2,:)+ data_xe(2,:), 'r-', 'LineWidth',1);
ylabel('$e_\theta$ $[rad]$','Interpreter','latex','FontSize',11)
xlabel('(b) Thời gian [s]')
subplot(3,1,3)
hold on; grid on;
plot(tspan, -data_xe_ref(3,:)+ data_xe(3,:), 'r-', 'LineWidth',1); 
ylabel('$e_\psi$ [m]','Interpreter','latex','FontSize',11); 
xlabel('(c) Thời gian [s]')

figure(4)
plot3(data_xp_ref(1,:), data_xp_ref(2,:), data_xp_ref(3,:), 'k-','LineWidth',1.0);
hold on; grid on; 
plot3(data_xp(1,:), data_xp(2,:), data_xp(3,:), 'r--','LineWidth',1.0); 
xlabel('$x$ $[m]$','Interpreter','latex','FontSize',11);
ylabel('$y$ $[m]$','Interpreter','latex','FontSize',11); 
zlabel('$z$ $[m]$','Interpreter','latex','FontSize',11); 

figure(5)
subplot(3,1,1)
hold on; grid on;
plot(tspan, -data_xp_ref(1,:)+ data_xp(1,:), 'r-', 'LineWidth',1); 
ylabel('$e_x$ $[m]$','Interpreter','latex','FontSize',11)
xlabel('(a) Thời gian [s]')
subplot(3,1,2)
hold on; grid on;
plot(tspan, -data_xp_ref(2,:)+ data_xp(2,:), 'r-', 'LineWidth',1);
ylabel('$e_y$ $[m]$','Interpreter','latex','FontSize',11)
xlabel('(b) Thời gian [s]')
subplot(3,1,3)
hold on; grid on;
plot(tspan, -data_xp_ref(3,:)+ data_xp(3,:), 'r-', 'LineWidth',1); 
ylabel('$e_z$ $[m]$','Interpreter','latex','FontSize',11); 
xlabel('(c) Thời gian [s]')




function result = trans_mat(xe)
    phi = xe(1); theta = xe(2); psi = xe(3);
    
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

function result = rot_mat(J, xe)
        
        Jx = J(1); Jy = J(2); Jz = J(3);
        phi = xe(1); dphi = xe(4); theta = xe(2); dtheta = xe(5); psi = xe(3); dpsi = xe(6);

        c11 = 0;
        c12 = (Jy- Jz)*(dtheta*cos(phi)*sin(phi) + dpsi*sin(phi)^2*cos(theta))/Jx ...
            + ((Jz- Jy)*dpsi*cos(phi)^2*cos(theta) - Jx*dpsi*cos(theta))/Jx;
        c13 = (Jz- Jy)*(dpsi*cos(phi)*sin(phi)*cos(theta)^2)/Jx;
        c21 = (Jz- Jy)*(dtheta*cos(phi)*sin(phi) + dpsi*sin(phi)^2*cos(theta))/Jy ...
            + ((Jy- Jz)*dpsi*cos(phi)^2*cos(theta) + Jx*dpsi*cos(theta))/Jy;
        c22 = (Jz- Jy)*dphi*cos(phi)*sin(phi)/Jy;
        c23 = (-Jx*dpsi*sin(theta)*cos(theta) + Jz*dpsi*cos(phi)^2*cos(theta)*sin(theta))/Jy ...
            + dpsi*sin(phi)*cos(theta)*sin(theta);
        c31 = ((Jy- Jz)*dpsi*cos(theta)^2*sin(phi)*cos(phi) - Jx*dtheta*cos(theta))/Jz;
        c32 = (Jz- Jy)*(dtheta*cos(phi)*sin(phi)*sin(theta) + dphi*sin(phi)^2*cos(theta))/Jz ...
            +((Jy- Jz)*dphi*cos(phi)^2*cos(theta) + Jx*dpsi*sin(theta)*cos(theta))/Jz ...
            - Jy*dpsi*sin(phi)^2*sin(theta)*cos(theta)/Jz - dpsi*cos(phi)^2*cos(theta)*sin(theta);
        c33 = (-Jz*dtheta*cos(phi)^2*sin(theta)*cos(theta) - Jy*dtheta*sin(phi)^2*cos(theta)*sin(theta))/Jz ...
            +((Jy- Jz)*dphi*cos(phi)*sin(phi)*cos(theta)^2 + Jx*dtheta*sin(theta)*cos(theta))/Jz;

        C = [c11 c12 c13; c21 c22 c23; c31 c32 c33];

        result = C;
end

%- e function
function result = e(n,m)
    temp = zeros(n,1);
    temp(m) = 1.0;
    result = temp;
end

function result = ds_fun(x, delta)
    [n, m] = size(x);
    temp = zeros(n, m);
    for i = 1:n
        for j = 1:m
            if abs(x(i,j)) > delta(i,j)
                temp(i,j) = 0.5*abs(x(i,j))^(-0.5);
            else
                temp(i,j) = delta(i,j)^(-0.5);
            end
        end
    end
    result = temp;
end













