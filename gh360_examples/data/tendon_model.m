data = readmatrix("elbow_tendon_model_data.csv");


motor_pos = data(:,3);
joint_pos = data(:,2);
%test = joint_pos./left_motor_pos;

r_active = 15.6;
r_passive = 40;

joint_vel = zeros(size(joint_pos)-1);
motor_vel_calc = zeros(size(motor_pos)-1);

for i=2:size(joint_vel)
    joint_vel(i-1) = (joint_pos(i) - joint_pos(i-1))/(data(i,1)-data(i-1,1));
    motor_vel_calc(i-1) = (motor_pos(i) - motor_pos(i-1))/(data(i,1)-data(i-1,1));
end
%normalise angle movement using pulley radius

%calculate strain using motor and joint position

joint_vel_adj = joint_vel * (r_passive/r_active);

hold on
plot(joint_vel)
plot(joint_vel_adj)
plot(motor_vel)
legend('joint vel', 'joint vel adj', 'motor vel')
hold off

%%
close all
clear all

data_2 = readmatrix("elbow_tendon_model_data_4.csv");

r_active = 15.6;
r_passive = 40;
r_tendon = 1.5;
l_free = 92.830733057538652;
alpha_passive_zero = 2.7936071344430391;
alpha_active_zero = 1.9522577849185718;
l_relaxed = 184.44311255778268;



start_index = 6;

time = data_2(:,1) - data_2(1,1);
time = time(start_index:size(data_2(:,1)),1);
joint_pos = data_2(start_index:size(data_2(:,1)),2);
joint_vel = data_2(start_index:size(data_2(:,1)),3);
motor_pos = data_2(start_index:size(data_2(:,1)),4);
motor_vel = data_2(start_index:size(data_2(:,1)),5);

joint_vel_filtered = smoothdata(joint_vel, 'movmean', 30); % through lp filter

r_active_adj = zeros(size(joint_pos));
r_active_adj_2 = zeros(size(joint_pos));
epsilon = 0.3927;
r_inc = 1.5;
thresh = 1;
for i=1:size(r_active_adj)
    alpha = alpha_active_zero + motor_pos(i);
    if alpha > 2*pi*thresh
        r_inc = r_inc + 3.0;
        thresh = thresh + 1 ;
    end
    r_active_adj(i) = r_active + r_inc;
    r_active_adj_2(i) = radius_ajd(r_active, alpha, r_tendon, epsilon);
end

strain = ((alpha_active_zero + motor_pos).*r_active_adj_2 + l_free + (joint_pos+alpha_passive_zero)*(r_passive+1.5))./l_relaxed - 1;

adj_factor = (r_passive+1.5)./r_active_adj_2;
joint_vel_adj = joint_vel_filtered .* adj_factor;
%joint_vel_filtered = smoothdata(joint_vel_adj, 'movmean', 30); % through lp filter
motor_vel_filtered = smoothdata(motor_vel, 'movmean', 30); % through lp filter

joint_vel_calc = zeros(size(joint_pos));

for i=2:size(joint_vel)
    joint_vel_calc(i) = (joint_pos(i) - joint_pos(i-1))/(data_2(i,1)-data_2(i-1,1));
end




figure;
%joint_vel_adj = joint_vel * (r_passive/r_active);
hold on
plot(time,joint_vel)
%plot(time,joint_vel_calc)
plot(time,joint_vel_adj)
plot(time,motor_vel)
plot(time,joint_vel_filtered)
plot(time,motor_vel_filtered)
legend('joint vel' ,'joint vel adj','motor vel', 'joint vel adj filtered', 'motor vel filtered')
xlabel('time [s]')
ylabel('velocity [rad/s]')
hold off


%joint_vel_adj = joint_vel * (r_passive/r_active);
figure;
hold on
%plot(time,joint_vel)
%plot(time,joint_vel_calc)
plot(joint_pos,joint_vel_adj)
plot(joint_pos,motor_vel)
%plot(joint_pos,joint_vel_filtered)
plot(joint_pos,motor_vel_filtered)
legend('joint vel adj','motor vel', 'joint vel adj filtered', 'motor vel filtered')
xlabel('joint position [rad]')
ylabel('velocity [rad/s]')
hold off

figure;
hold on
%plot(time,joint_vel)
%plot(time,joint_vel_calc)
plot(strain,joint_vel_adj)
plot(strain,motor_vel)
%plot(joint_pos,joint_vel_filtered)
plot(strain,motor_vel_filtered)
legend('joint vel adj','motor vel', 'joint vel adj filtered', 'motor vel filtered')
xlabel('strain [%]')
ylabel('velocity [rad/s]')
hold off

%%
close all
%clear all

tendon_vel = joint_vel_filtered * (r_passive+1.5);
motor_tendon_vel = motor_vel_filtered .* r_active_adj_2;


figure;
hold on
plot(time,strain)
xlabel('time [s]')
ylabel('strain [%]')
hold off

figure;
hold on
plot(joint_pos,strain)
xlabel('jont position [rad]')
ylabel('strain [%]')
hold off

figure;
hold on
plot(time,tendon_vel)
plot(time,motor_tendon_vel)
xlabel('time [s]')
ylabel('tendon velocity [mm/s]')
hold off

figure;
hold on
plot(strain,tendon_vel)
plot(strain,motor_tendon_vel)
xlabel('time [s]')
ylabel('tendon velocity [mm/s]')
hold off

%%
close all;

angle = 0.0:0.01:6*pi;
radius = zeros(size(angle'));

for i=1:size(angle')
    radius(i) = radius_ajd(40, angle(i), 1.5, 0.3927);
end

figure;
plot(angle,radius)
xlabel('angle [rad]')
ylabel('adjusted pulley radius [mm]')


function ret = radius_ajd(r_pulley, angle, r_tendon, epsilon)
    
    ret = (r_pulley+r_tendon) +  smoothing_function(angle,epsilon,2*pi)*(r_tendon*2) + smoothing_function(angle,epsilon,4*pi)*(r_tendon*2);
end

function ret = smoothing_function(angle, epsilon, center)
    ret = 1 / (1+exp(-1/epsilon^2 * (angle-center)));
end


