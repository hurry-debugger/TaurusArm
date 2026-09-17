function q_sol = my_Inverse_solution(robot, T_target)

    T_basic_to_joint5 = T_target * inv(robot.tool.T);

    R_target = T_basic_to_joint5(1:3, 1:3);
    P_target = T_basic_to_joint5(1:3, 4);

    h  = robot.links(1).d;
    l1 = robot.links(2).a;
    l2 = robot.links(4).d;
    l3 = robot.links(6).d;

    P2_pos = P_target - l3 * R_target(:, 3);
    P2x = P2_pos(1); P2y = P2_pos(2); P2z = P2_pos(3);


    q0 = atan2(P2y, P2x);

    r = sqrt(P2x^2 + P2y^2);
    s = P2z - h;


    P0P2_sq = r^2 + s^2;
    P0P2 = sqrt(P0P2_sq);


    if P0P2 > (l1 + l2) || P0P2 < abs(l1 - l2)
        error('目标超出工作空间');
    end


    phi1 = atan2(s, r);

    phi2 = acos((l1^2 + P0P2_sq - l2^2) / (2 * l1 * P0P2));

    phi4 = acos((l1^2 + l2^2 - P0P2_sq) / (2 * l1 * l2));

    q1 = pi/2 - (phi1 + phi2);

    q2 = pi - phi4;

    T01 = robot.links(1).A(q0);
    T12 = robot.links(2).A(q1);
    T23 = robot.links(3).A(q2);

    T03 = T01 * T12 * T23;
    R03 = T03.R;

    R36 = R03' * R_target;

    r13 = R36(1,3); r23 = R36(2,3); r33 = R36(3,3);
    r32 = R36(3,2); r31 = R36(3,1);

    normalize = @(angle) mod(angle + pi, 2*pi) - pi;


    j5_s1 = atan2(sqrt(r13^2 + r23^2), r33);
    if abs(sin(j5_s1)) > 1e-5
        j4_s1 = atan2(-r23, -r13);
        j6_s1 = atan2(-r32, r31);
    else
        j4_s1 = 0; j6_s1 = atan2(-R36(1,2), R36(1,1));
    end


    j5_s2 = -j5_s1;
    j4_s2 = j4_s1 + pi;
    j6_s2 = j6_s1 - pi;
    j6_min = robot.links(6).qlim(1);
    j6_max = robot.links(6).qlim(2);
    j6_candidate_1 = normalize(j6_s1 - robot.links(6).offset);


    if j6_candidate_1 >= j6_min && j6_candidate_1 <= j6_max

        q_final = [q0, q1, q2, j4_s1, j5_s1, j6_s1];
    else
        q_final = [q0, q1, q2, j4_s2, j5_s2, j6_s2];
    end


    q_sol = zeros(1,6);

    offsets = [0, 0, 0, robot.links(4).offset, robot.links(5).offset, robot.links(6).offset];

    for i = 1:6
        q_sol(i) = normalize(q_final(i) - offsets(i));
    end
end
