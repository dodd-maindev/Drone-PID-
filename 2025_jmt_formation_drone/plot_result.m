close all;

data_p1 = zeros(3, numdata);
data_p2 = zeros(3, numdata);
data_p3 = zeros(3, numdata);
data_p4 = zeros(3, numdata);

data_p1_des = zeros(3, numdata);
data_p2_des = zeros(3, numdata);
data_p3_des = zeros(3, numdata);
data_p4_des = zeros(3, numdata);

data_eta_1 = zeros(3, numdata);
data_eta_2 = zeros(3, numdata);
data_eta_3 = zeros(3, numdata);
data_eta_4 = zeros(3, numdata);

data_eta_ref_1 = zeros(3, numdata);
data_eta_ref_2 = zeros(3, numdata);
data_eta_ref_3 = zeros(3, numdata);
data_eta_ref_4 = zeros(3, numdata);

data_eta_des_1 = zeros(3, numdata);
data_eta_des_2 = zeros(3, numdata);
data_eta_des_3 = zeros(3, numdata);
data_eta_des_4 = zeros(3, numdata);

for i = 1:numdata
    data_p1(:,i) = p_i{1,i};
    data_p2(:,i) = p_i{2,i};
    data_p3(:,i) = p_i{3,i};
    data_p4(:,i) = p_i{4,i};

    data_p1_des(:,i) = p_des_i{1,i};
    data_p2_des(:,i) = p_des_i{2,i};
    data_p3_des(:,i) = p_des_i{3,i};
    data_p4_des(:,i) = p_des_i{4,i};

    data_eta_1(:,i) = eta_i{1,i};
    data_eta_2(:,i) = eta_i{2,i};
    data_eta_3(:,i) = eta_i{3,i};
    data_eta_4(:,i) = eta_i{4,i};

    data_eta_des_1(:,i) = eta_des_i{1,i};
    data_eta_des_2(:,i) = eta_des_i{2,i};
    data_eta_des_3(:,i) = eta_des_i{3,i};
    data_eta_des_4(:,i) = eta_des_i{4,i};

    data_eta_ref_1(:,i) = eta_ref_i{1,i};
    data_eta_ref_2(:,i) = eta_ref_i{2,i};
    data_eta_ref_3(:,i) = eta_ref_i{3,i};
    data_eta_ref_4(:,i) = eta_ref_i{4,i};
end




figure(1)
subplot(3,1,1)
hold on
plot(tspan, data_p1_des(1,:), '-');
plot(tspan, data_p2_des(1,:), '--');
plot(tspan, data_p3_des(1,:), '-');
plot(tspan, data_p4_des(1,:), '--');
subplot(3,1,2)
hold on
plot(tspan, data_p1_des(2,:), '-');
plot(tspan, data_p2_des(2,:), '--');
plot(tspan, data_p3_des(2,:), '-');
plot(tspan, data_p4_des(2,:), '--');
subplot(3,1,3)
hold on
plot(tspan, data_p1_des(3,:), '-');
plot(tspan, data_p2_des(3,:), '--');
plot(tspan, data_p3_des(3,:), '-');
plot(tspan, data_p4_des(3,:), '--');


figure(2)
subplot(3,1,1)
hold on
plot(tspan, data_p1(1,:), '-');
plot(tspan, data_p2(1,:), '--');
plot(tspan, data_p3(1,:), '-');
plot(tspan, data_p4(1,:), '--');
subplot(3,1,2)
hold on
plot(tspan, data_p1(2,:), '-');
plot(tspan, data_p2(2,:), '--');
plot(tspan, data_p3(2,:), '-');
plot(tspan, data_p4(2,:), '--');
subplot(3,1,3)
hold on
plot(tspan, data_p1(3,:), '-');
plot(tspan, data_p2(3,:), '--');
plot(tspan, data_p3(3,:), '-');
plot(tspan, data_p4(3,:), '--');

figure(3)
subplot(3,1,1)
hold on
plot(tspan, data_eta_1(1,:), '-');
plot(tspan, data_eta_2(1,:), '--');
plot(tspan, data_eta_3(1,:), '-');
plot(tspan, data_eta_4(1,:), '--');
subplot(3,1,2)
hold on
plot(tspan, data_eta_1(2,:), '-');
plot(tspan, data_eta_2(2,:), '--');
plot(tspan, data_eta_3(2,:), '-');
plot(tspan, data_eta_4(2,:), '--');
subplot(3,1,3)
hold on
plot(tspan, data_eta_1(3,:), '-');
plot(tspan, data_eta_2(3,:), '--');
plot(tspan, data_eta_3(3,:), '-');
plot(tspan, data_eta_4(3,:), '--');

figure(4)
subplot(3,1,1)
hold on
plot(tspan, data_eta_ref_1(1,:), '-');
plot(tspan, data_eta_ref_2(1,:), '--');
plot(tspan, data_eta_ref_3(1,:), '-');
plot(tspan, data_eta_ref_4(1,:), '--');
subplot(3,1,2)
hold on
plot(tspan, data_eta_ref_1(2,:), '-');
plot(tspan, data_eta_ref_2(2,:), '--');
plot(tspan, data_eta_ref_3(2,:), '-');
plot(tspan, data_eta_ref_4(2,:), '--');
subplot(3,1,3)
hold on
plot(tspan, data_eta_ref_1(3,:), '-');
plot(tspan, data_eta_ref_2(3,:), '--');
plot(tspan, data_eta_ref_3(3,:), '-');
plot(tspan, data_eta_ref_4(3,:), '--');


figure(5)
subplot(3,1,1)
hold on
plot(tspan, data_eta_des_1(1,:), '-');
plot(tspan, data_eta_des_2(1,:), '--');
plot(tspan, data_eta_des_3(1,:), '-');
plot(tspan, data_eta_des_4(1,:), '--');
subplot(3,1,2)
hold on
plot(tspan, data_eta_des_1(2,:), '-');
plot(tspan, data_eta_des_2(2,:), '--');
plot(tspan, data_eta_des_3(2,:), '-');
plot(tspan, data_eta_des_4(2,:), '--');
subplot(3,1,3)
hold on
plot(tspan, data_eta_des_1(3,:), '-');
plot(tspan, data_eta_des_2(3,:), '--');
plot(tspan, data_eta_des_3(3,:), '-');
plot(tspan, data_eta_des_4(3,:), '--');