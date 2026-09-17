function info = get_target_description(varargin)

    if isempty(varargin)
        error('必须输入参数');
    end
    firstArg = varargin{1};
    args_to_parse = {};

    if isfloat(firstArg) && ismatrix(firstArg) && all(size(firstArg) == [4,4])
        T_World_to_E = firstArg;
        if length(varargin) > 1
            args_to_parse = varargin(2:end);
        end

    elseif isfloat(firstArg) && isscalar(firstArg) && length(varargin) >= 6
        x = varargin{1}; y = varargin{2}; z = varargin{3};
        theta = varargin{4}; phi = varargin{5}; alpha = varargin{6};

        validate_target_params(x, y, z, theta, phi, alpha);

        E_pos = [x, y, z];

        R_E = trotz(deg2rad(theta)) * ...
              troty(deg2rad(-(90-phi))) * ...
              trotx(deg2rad(alpha));
        T_World_to_E = transl(E_pos) * R_E;

        if length(varargin) > 6
            args_to_parse = varargin(7:end);
        end
    else
        error('输入格式无法识别');
    end

    p = inputParser;
    p.CaseSensitive = false;
    p.KeepUnmatched = true;

    addParameter(p, 'Stage', 'Default');
    addParameter(p, 'TotalFrame', 30);
    addParameter(p, 'CurrentFrame', 1);
    addParameter(p, 'Q_rot_angle', 30);
    addParameter(p, 'start_z_rotate_angle', 0);

    parse(p, args_to_parse{:});

    current_stage = p.Results.Stage;
    Total_Frame   = p.Results.TotalFrame;
    Current_Frame = p.Results.CurrentFrame;
    Q_rot_angle   = p.Results.Q_rot_angle;
    start_z_rotate_angle = p.Results.start_z_rotate_angle;
    Current_Frame = max(1, min(Current_Frame, Total_Frame));

    start_Pos_in_E = [-0.025; 0; 0];
    start_Rot_in_E = trotz(deg2rad(90)) * trotx(deg2rad(90)) * trotz(deg2rad(start_z_rotate_angle));
    T_E_to_Start   = transl(start_Pos_in_E) * start_Rot_in_E;

    Transl_finish_Pos_in_E = [0; 0; 0.1];
    T_E_to_Transl_finish   = transl(Transl_finish_Pos_in_E);

    info.T_E_to_Transl_finish = T_E_to_Transl_finish;

    P_vector_in_E = [0; -1; 0];
    P_point_in_E  = [-0.169; 0.0; 0.2085];

    T_E_to_P_Rot_finish = compute_rotated_pose(T_E_to_Transl_finish, P_vector_in_E, P_point_in_E, 90);

    info.T_E_to_P_Rot_finish = T_E_to_P_Rot_finish;

    Q_vector_in_E = [1; 0; 0];
    Q_point_in_E  = [0.0; 0.0; 0.2085+0.015];

    T_E_to_Q_Rot_finish = compute_rotated_pose(T_E_to_P_Rot_finish, Q_vector_in_E, Q_point_in_E, Q_rot_angle);

    info.T_E_to_Q_Rot_finish = T_E_to_Q_Rot_finish;

    info.T_E_to_Start         = T_E_to_Start;
    info.T_World_to_E         = T_World_to_E;
    info.T_World_to_Start     = T_World_to_E * T_E_to_Start;
    info.T_World_to_Step3     = T_World_to_E * T_E_to_Transl_finish * T_E_to_Start;
    info.T_World_to_Step4     = T_World_to_E * T_E_to_P_Rot_finish * T_E_to_Start;
    info.T_World_to_Step5     = T_World_to_E * T_E_to_Q_Rot_finish * T_E_to_Start;

    info.P_in_World.point  = apply_transform(T_World_to_E, P_point_in_E);
    info.P_in_World.vector = T_World_to_E(1:3, 1:3) * P_vector_in_E;

    info.Q_in_World.point  = apply_transform(T_World_to_E, Q_point_in_E);
    info.Q_in_World.vector = T_World_to_E(1:3, 1:3) * Q_vector_in_E;

    t = Current_Frame / Total_Frame;
    switch current_stage
        case 'Start'
            T_E_to_Target_Local = T_E_to_Start;
            info.T_World_to_Target = info.T_World_to_Start;
        case 'Transl'
            current_dist = 0.1 * t;
            Current_Pos = [0; 0; current_dist];
            T_E_to_Target_Local = transl(Current_Pos);
            info.T_World_to_Target = T_World_to_E * T_E_to_Target_Local * T_E_to_Start;
        case 'P_Rotate'
            current_ang = 90.0 * t;
            T_E_to_Target_Local = compute_rotated_pose(T_E_to_Transl_finish, P_vector_in_E, P_point_in_E, current_ang);
            info.T_World_to_Target = T_World_to_E * T_E_to_Target_Local * T_E_to_Start;

        case 'Q_Rotate'
            current_ang = Q_rot_angle * t;
            T_E_to_Target_Local = compute_rotated_pose(T_E_to_P_Rot_finish, Q_vector_in_E, Q_point_in_E, current_ang);
            info.T_World_to_Target = T_World_to_E * T_E_to_Target_Local * T_E_to_Start;

        otherwise
            info.T_World_to_Target = T_World_to_E * T_E_to_Start;
    end

    info.T_E_to_Target = T_E_to_Target_Local;
end

function T_new = compute_rotated_pose(T_current, axis_vec, pivot_point, theta_deg)

    theta = deg2rad(theta_deg);
    k = axis_vec / norm(axis_vec);

    K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
    I = eye(3);


    R_rot = I + sin(theta) * K + (1 - cos(theta)) * (K * K);

    pivot_point = pivot_point(:);


    t_part = (I - R_rot) * pivot_point;


    T_action = eye(4);
    T_action(1:3, 1:3) = R_rot;
    T_action(1:3, 4)   = t_part;


    T_new = T_action * T_current;
end

function p_world = apply_transform(T, p_local)
    p_4d = T * [p_local(:); 1];
    p_world = p_4d(1:3);
end

function validate_target_params(x, y, z, theta, phi, alpha)

    Lim.x = [-0.100, 0.000];
    Lim.y = [ 0.100, 0.300];
    Lim.z = [ 0.500, 0.700];
    Lim.theta = [-90, 90];
    Lim.phi   = [0, 90];
    Lim.alpha = [-45, 45];

    check_one('x', x, Lim.x);
    check_one('y', y, Lim.y);
    check_one('z', z, Lim.z);
    check_one('theta', theta, Lim.theta);
    check_one('phi', phi, Lim.phi);
    check_one('alpha', alpha, Lim.alpha);
end

function check_one(name, val, range)
    if val < range(1) || val > range(2)
        error('参数异常: "%s" 的值 (%.2f) 超出允许范围', name, val);
    end
end

function plot_target_scene(info, axis_len)

    if nargin < 2, axis_len = 0.05; end
    frames_to_plot = {
        info.T_World_to_E,     'E',      2.0, '-';
        info.T_World_to_Start, 'Start',  1.0, '--';
        info.T_World_to_Step3, 'S3',     0.5, ':';
        info.T_World_to_Step4, 'S4',     0.5, ':';
        info.T_World_to_Step5, 'S5',     0.5, ':';
    };

    for i = 1:size(frames_to_plot, 1)
        T     = frames_to_plot{i, 1};
        name  = frames_to_plot{i, 2};
        thick = frames_to_plot{i, 3};
        style = frames_to_plot{i, 4};

        draw_single_frame(T, name, axis_len, thick, style);
    end

    draw_rotation_axis(info.P_in_World, 'Axis-P', 'm', 0.3);


    draw_rotation_axis(info.Q_in_World, 'Axis-Q', 'c', 0.3);

end

function draw_single_frame(T, name, len, thick, style)
    origin = T(1:3, 4);
    x_tip  = origin + T(1:3, 1) * len;
    y_tip  = origin + T(1:3, 2) * len;
    z_tip  = origin + T(1:3, 3) * len;

    hold on;
    plot3([origin(1) x_tip(1)], [origin(2) x_tip(2)], [origin(3) x_tip(3)], 'r', 'LineWidth', thick, 'LineStyle', style);
    plot3([origin(1) y_tip(1)], [origin(2) y_tip(2)], [origin(3) y_tip(3)], 'g', 'LineWidth', thick, 'LineStyle', style);
    plot3([origin(1) z_tip(1)], [origin(2) z_tip(2)], [origin(3) z_tip(3)], 'b', 'LineWidth', thick, 'LineStyle', style);


    text(origin(1), origin(2), origin(3), name, 'FontSize', 9, 'Interpreter', 'none');
end

function draw_rotation_axis(axis_info, name, color_char, draw_len)
    pt = axis_info.point;
    vec = axis_info.vector;
    end_pt = pt + vec * draw_len;

    plot3([pt(1) end_pt(1)], [pt(2) end_pt(2)], [pt(3) end_pt(3)], ...
          [color_char '--'], 'LineWidth', 1.5);
    text(end_pt(1), end_pt(2), end_pt(3), name, 'Color', color_char);
end