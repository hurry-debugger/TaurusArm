function robot = get_robot_description()

    h = 0.35;

    l1 = 0.38;

    l2 = 0.44;

    l3 = 0.01;

    gripper_len = 0.13;

    L(1) = Link([0, h, 0, -pi/2], 'standard');                      L(1).qlim = [-150, 150]*pi/180;
    L(2) = Link([0, 0, l1, 0], 'standard');     L(2).offset = -pi/2; L(2).qlim = [-90, 45]*pi/180;
    L(3) = Link([0, 0, 0, -pi/2], 'standard');   L(3).offset = -pi/2; L(3).qlim = [40, 170]*pi/180;
    L(4) = Link([0, l2, 0, pi/2], 'standard');                       L(4).qlim = [-400, 400]*pi/180;
    L(5) = Link([0, 0, 0, -pi/2], 'standard');                              L(5).qlim = [-130, 130]*pi/180;
    L(6) = Link([0, l3, 0, 0], 'standard');     L(6).offset = pi; L(6).qlim = [-170, 170]*pi/180;

    robot = SerialLink(L, 'name', 'RM_Engineer');
    robot.tool = transl(0, 0, gripper_len) * troty(deg2rad(-90));
end