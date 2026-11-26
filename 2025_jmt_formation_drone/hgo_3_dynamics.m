function result = hgo_3_dynamics(t, input)
    y = input(1:3);
    x = input(4:6);
    dx = input(7:9);
    ddx = input(10:12);

    c1 = 3;
    c2 = 3;
    c3 = 1;
    epsilon = 0.01;

    x_1dot = dx+ c1*(y - x)/epsilon;
    x_2dot = ddx+ c2*(y - x)/epsilon^2;
    x_3dot = c3*(y - x)/epsilon^3;

    result = [x; x_1dot; x_2dot; x_3dot];
end