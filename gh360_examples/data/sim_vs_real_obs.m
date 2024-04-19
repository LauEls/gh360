close all
clear all

sim_obs = readmatrix("rollout_obs_data_rollout.csv");
real_obs = readmatrix("rollout_obs_data_learning.csv");

%% Oservations
% door_pos -> 1 - 3
% handle_pos -> 4 - 6
% handle_to_eef_pos -> 7 - 9
% hinge_qpos -> 10
% handle_qpos -> 11
% joint_pos -> 12 - 18
% joint_vel -> 19 - 25
% (motor_pos) -> 26 - 37
% eef_pos -> 38 - 40
% eef_quat - 41 - 44


%% EEF Position

for i=1:3
    
end


%% Joint Posiiton
close all

for i=1:7
    pos_sim = sim_obs(:,i+11);
    pos_real = real_obs(:,i+11);

    figure;
    hold on
    plot(pos_sim)
    plot(pos_real)
    ylabel("joint angle [rad]")
    xlabel("set iteration")
    if i == 1
        title("Shoulder Yaw")
    elseif i == 2
        title("Shoulder Roll")
    elseif i == 3
        title("Shoulder Pitch")
    elseif i == 4
        title("Upperarm Roll")
    elseif i == 5
        title("Elbow")
    elseif i == 6
        title("Forearm Roll")
    elseif i == 7
        title("Wrist Pitch")
    end
    hold off
end

%% Joint Velocity
close all

for i=1:7
    vel_sim = sim_obs(:,i+18);
    vel_real = real_obs(:,i+18);

    figure;
    hold on
    plot(vel_sim)
    plot(vel_real)
    ylabel("joint velocity [rad/s]")
    xlabel("set iteration")
    if i == 1
        title("Shoulder Yaw")
    elseif i == 2
        title("Shoulder Roll")
    elseif i == 3
        title("Shoulder Pitch")
    elseif i == 4
        title("Upperarm Roll")
    elseif i == 5
        title("Elbow")
    elseif i == 6
        title("Forearm Roll")
    elseif i == 7
        title("Wrist Pitch")
    end
    hold off
end

%% Motor Position
close all

for i=1:13
    vel_sim = sim_obs(:,i+25);
    vel_real = real_obs(:,i+25);

    figure;
    hold on
    plot(vel_sim)
    plot(vel_real)
    ylabel("Motor Position [rad]")
    xlabel("set iteration")
    title("Motor "+string(i))
    hold off
end

%%
i = 1;

pos_sim = sim_obs(:,i+11);
pos_real = real_obs(:,i+11);

figure;
hold on
plot(pos_sim)
plot(pos_real)
ylabel("joint angle [rad]")
xlabel("set iteration")
if i == 1
    title("Shoulder Yaw")
end

vel_sim = sim_obs(:,i+18);
vel_real = real_obs(:,i+18);

figure;
hold on
plot(vel_sim)
plot(vel_real)
ylabel("joint velocity [rad/s]")
xlabel("set iteration")
if i == 1
    title("Shoulder Yaw")
end


vel_sim = sim_obs(:,i+25);
vel_real = real_obs(:,i+25);

figure;
hold on
plot(vel_sim)
plot(vel_real)
ylabel("Motor Position [rad]")
xlabel("set iteration")
title("Motor "+string(i))
hold off

vel_sim = sim_obs(:,i+1+25);
vel_real = real_obs(:,i+1+25);

figure;
hold on
plot(vel_sim)
plot(vel_real)
ylabel("Motor Position [rad]")
xlabel("set iteration")
title("Motor "+string(i))
hold off
