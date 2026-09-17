function trajectory_data = calculate_scene_trajectory(E_pos, E_euler, Base_Pos, robot_in, varargin)

    fr_trans = 10;
    fr_rotp = 10;
    fr_rotq = 10;
    start_z_vec = 0:10:355;
    q_angle_total = -50;
    new_l1 = [];
    new_l2 = [];

    for i = 1:2:length(varargin)
        val = varargin{i+1};
        switch varargin{i}
            case 'Frames_Transl', fr_trans = val;
            case 'Frames_RotP',   fr_rotp = val;
            case 'Frames_RotQ',   fr_rotq = val;
            case 'Override_L1',   new_l1 = val;
            case 'Override_L2',   new_l2 = val;
            case 'Angle_Q_Total', q_angle_total = val;
            case 'Start_Z_Angle', start_z_vec = val;
        end
    end

    robot = robot_in;

    if ~isempty(new_l1)
        robot.links(2).a = new_l1;
    end

    if ~isempty(new_l2)
        robot.links(4).d = new_l2;
    end

    limits = robot.qlim;
    if isempty(limits), limits = repmat([-pi, pi], 6, 1); end


    base_rpy = [180, 0, 0];
    T_World_to_Base = transl(Base_Pos) * ...
                      trotz(deg2rad(base_rpy(1))) * ...
                      troty(deg2rad(base_rpy(2))) * ...
                      trotx(deg2rad(base_rpy(3)));

    ex = E_pos(1); ey = E_pos(2); ez = E_pos(3);
    eth = E_euler(1); eph = E_euler(2); eal = E_euler(3);

    total_frames = fr_trans + fr_rotp + fr_rotq;
    trajectory_data = zeros(total_frames, 9);
    row_idx = 0;


    stages = {'Transl', fr_trans; 'P_Rotate', fr_rotp; 'Q_Rotate', fr_rotq};

    for s = 1:3
        stg_name = stages{s, 1};
        n_frames = stages{s, 2};

        for i = 1 : n_frames
            row_idx = row_idx + 1;


            [best_q, best_s, success] = find_first_valid_q(robot, limits, T_World_to_Base, start_z_vec, ...
                ex, ey, ez, eth, eph, eal, stg_name, i, n_frames, q_angle_total);


            if ~success
                trajectory_data = [];
                return;
            end

            trajectory_data(row_idx, :) = [s, i, best_s, best_q];
        end
    end
end


function [final_q_deg, final_s_val, is_success] = find_first_valid_q(robot, limits, T_Base, search_angles, ...
                                            ex, ey, ez, eth, eph, eal, ...
                                            stg_name, cur_f, tot_f, q_ang)

    is_success = false;
    final_q_deg = [NaN, NaN, NaN, NaN, NaN, NaN];
    final_s_val = NaN;

    for s_angle = search_angles

        info = get_target_description(ex, ey, ez, eth, eph, eal, ...
            'Stage', stg_name, 'Q_rot_angle', q_ang, ...
            'CurrentFrame', cur_f, 'TotalFrame', tot_f, ...
            'start_z_rotate_angle', s_angle);

        T_Target = info.T_World_to_Target;
        T_in_Base = T_Base \ T_Target;

        try
            q = my_Inverse_solution(robot, T_in_Base);

            if isempty(q) || any(isnan(q)), continue; end

            if any(q < limits(:,1)' - 1e-3) || any(q > limits(:,2)' + 1e-3)
                continue;
            end

            final_q_deg = rad2deg(q);
            final_s_val = s_angle;
            is_success = true;
            return;

        catch
            continue;
        end
    end
end