function Results_Matrix = run_arm_optimization()

    clc; close all;

    L1_Range = 0.36 : 0.1 : 0.48;
    L2_Range = 0.38 : 0.1 : 0.50;

    Angle_Q_Total = -50;

    Base_X = 0.2;
    Base_Z = 0.0;
    Base_Y_List  = -0.1 : 0.1 : 0.2;
    Num_Base_Config = length(Base_Y_List);

    X_L = -0.1 : 0.1 : 0.0;
    Y_L = [ 0.2 ];
    Z_L = 0.50 : 0.1 : 0.70;
    TH_L = -80 : 30 : 0;
    PH_L =  20 : 30 : 80;
    AL_L = -45 : 30 : 45;

    [x_g, y_g, z_g, th_g, ph_g, al_g] = ndgrid(X_L, Y_L, Z_L, TH_L, PH_L, AL_L);
    Tasks = [x_g(:), y_g(:), z_g(:), th_g(:), ph_g(:), al_g(:)];
    Num_Tasks = size(Tasks, 1);


    [L1_Grid, L2_Grid] = ndgrid(L1_Range, L2_Range);
    Configs = [L1_Grid(:), L2_Grid(:)];
    Num_Configs = size(Configs, 1);


    robot_proto = get_robot_description();
    Limits_Rad = zeros(6,2);
    for i=1:6, Limits_Rad(i,:) = robot_proto.links(i).qlim; end
    if all(Limits_Rad == 0, 'all'), Limits_Rad = repmat([-pi, pi], 6, 1); end

    fprintf('==========================================================\n');
    fprintf('🔥 终极选型启动 (World Frame Tasks Confirmed) 🔥\n');
    fprintf('计算规模: %d 杆长组合 x %d 任务点 x %d 基座位置\n', Num_Configs, Num_Tasks, Num_Base_Config);
    fprintf('总计校验次数: %d 次\n', Num_Configs * Num_Tasks * Num_Base_Config);
    fprintf('==========================================================\n');


    if isempty(gcp('nocreate')), parpool; end
    start_tic = tic;

    Res_List = zeros(Num_Configs, 5);

    parfor k = 1 : Num_Configs
        l1 = Configs(k, 1);
        l2 = Configs(k, 2);

        valid_task_count = 0;
        base_adaptability_sum = 0;
        perfect_points_count = 0;


        robot_worker = robot_proto;

        for t = 1 : Num_Tasks
            pt = Tasks(t, 1:3);
            rot = Tasks(t, 4:6);

            valid_bases_for_this_task = 0;

            for by = Base_Y_List
                base_pos = [Base_X, by, Base_Z];

                is_success = check_feasibility_wrapper(l1, l2, Angle_Q_Total, ...
                                                       pt, rot, base_pos, Limits_Rad, robot_worker);
                if is_success
                    valid_bases_for_this_task = valid_bases_for_this_task + 1;
                end
            end


            if valid_bases_for_this_task > 0
                valid_task_count = valid_task_count + 1;

                base_adaptability_sum = base_adaptability_sum + (valid_bases_for_this_task / Num_Base_Config);

                if valid_bases_for_this_task == Num_Base_Config
                    perfect_points_count = perfect_points_count + 1;
                end
            end
        end


        cov = valid_task_count / Num_Tasks;

        if valid_task_count > 0
            avg_ada = base_adaptability_sum / valid_task_count;
        else
            avg_ada = 0;
        end

        perf_ratio = perfect_points_count / Num_Tasks;

        Res_List(k, :) = [l1, l2, cov, avg_ada, perf_ratio];


        fprintf('-> (%3d/%d) L1=%.2f, L2=%.2f | Cov: %5.1f%% | Ada: %.2f | Perf: %5.1f%%\n', ...
            k, Num_Configs, l1, l2, cov*100, avg_ada, perf_ratio*100);
    end

    total_time = toc(start_tic);

    Results_Matrix = Res_List;


    [~, sort_idx] = sortrows(Results_Matrix, [-3, -4, -5]);
    best = Results_Matrix(sort_idx(1), :);

    fprintf('\n========== 🏁 运算结束 (%.1f s) ==========\n', total_time);
    fprintf('🏆 最佳杆长: L1=%.2f, L2=%.2f\n', best(1), best(2));
    fprintf('   1. 覆盖率 (Coverage):     %5.1f%% (%d/%d)\n', best(3)*100, round(best(3)*Num_Tasks), Num_Tasks);
    fprintf('   2. 灵活性 (Adaptability): %.2f   (0-1)\n', best(4));
    fprintf('   3. 满分率 (Perfect Rate): %5.1f%% (%d/%d)\n', best(5)*100, round(best(5)*Num_Tasks), Num_Tasks);


    plot_results(Results_Matrix, L1_Range, L2_Range);
end


function is_ok = check_feasibility_wrapper(l1, l2, Angle_Q, pos, rot, base, limits_rad, robot_in)
    is_ok = false;

    search_vec = 0 : 30 : 330;
    frames_rough = 5;

    try

        traj = calculate_scene_trajectory(pos, rot, base, robot_in, ...
            'Frames_Transl', frames_rough, ...
            'Frames_RotP',   frames_rough, ...
            'Frames_RotQ',   frames_rough, ...
            'Override_L1',   l1, ...
            'Override_L2',   l2, ...
            'Angle_Q_Total', Angle_Q, ...
            'Start_Z_Angle', search_vec);


        if isempty(traj)
            is_ok = false;
            return;
        end

        if size(traj, 2) >= 9
            q_vals_deg = traj(:, 4:9);
        else
            q_vals_deg = traj(:, end-5:end);
        end


        if any(isnan(q_vals_deg), 'all')
            is_ok = false;
            return;
        end

        q_vals_rad = deg2rad(q_vals_deg);

        pass_limits = true;
        for j = 1:6

            if any(q_vals_rad(:,j) < limits_rad(j,1) - 0.01) || ...
               any(q_vals_rad(:,j) > limits_rad(j,2) + 0.01)
                pass_limits = false;
                break;
            end
        end

        if pass_limits
            is_ok = true;
        end

    catch
        is_ok = false;
    end
end


function plot_results(Res, L1_R, L2_R)

    rows = length(L1_R);
    cols = length(L2_R);

    Cov  = reshape(Res(:,3), rows, cols);
    Ada  = reshape(Res(:,4), rows, cols);
    Perf = reshape(Res(:,5), rows, cols);

    figure('Name', 'Optimization Results', 'Color','w', 'Position', [100,200,1600,500]);

    x_labels = string(L1_R);
    y_labels = string(L2_R);


    subplot(1, 3, 1);
    h1 = heatmap(x_labels, y_labels, Cov');
    title('1. 覆盖率 (Coverage)');
    xlabel('L1 Length (m)'); ylabel('L2 Length (m)');
    colormap(h1, jet);

    subplot(1, 3, 2);
    h2 = heatmap(x_labels, y_labels, Ada');
    title('2. 灵活性 (Adaptability)');
    xlabel('L1 Length (m)'); ylabel('L2 Length (m)');
    colormap(h2, parula);


    subplot(1, 3, 3);
    h3 = heatmap(x_labels, y_labels, Perf');
    title('3. 完美率 (Perfect Rate)');
    xlabel('L1 Length (m)'); ylabel('L2 Length (m)');
    colormap(h3, autumn);
end
