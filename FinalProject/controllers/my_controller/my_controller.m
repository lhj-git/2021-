desktop;

keyboard;

TIME_STEP = 32;
size_of_picture = 10;%定义图的大小，单位为米。必须和webots上的数据一致
size_of_grid = 1000;%定义栅格图的大小
limit = 5;%感知用参数
random_delete_number = 10;%感知用参数
discount_rate = 0.2;%感知用参数
leastI=0.4;

points_per_meter = size_of_grid/size_of_picture;
avg_grid = zeros(size_of_grid,size_of_grid);
show = zeros(size_of_grid,size_of_grid);
motor1 = wb_robot_get_device('motor1');
motor2 = wb_robot_get_device('motor2');
motor3 = wb_robot_get_device('motor3');
motor4 = wb_robot_get_device('motor4');
Keyboard = wb_robot_get_device('Keyboard');
Inertial_unit = wb_robot_get_device('inertial unit');
Lidar = wb_robot_get_device('lidar');
GPS = wb_robot_get_device('gps');

wb_motor_set_position(motor1,inf);
wb_motor_set_position(motor2,inf);
wb_motor_set_position(motor3,inf);
wb_motor_set_position(motor4,inf);
velocity = 10.0;
speed_forward = [ velocity ,velocity ,velocity ,velocity ];
speed_backward = [ -velocity ,-velocity ,-velocity ,-velocity ];
speed_leftward = [ velocity ,-velocity ,velocity ,-velocity ];
speed_rightward = [ -velocity ,velocity ,-velocity ,velocity ];

speed_leftCircle = [ velocity ,-velocity ,-velocity ,velocity ];
speed_rightCircle = [ -velocity ,velocity ,velocity ,-velocity ];
filter = ones(3,3)./8;
filter(2,2) = 0;
speed1 = zeros(4);
speed2 = zeros(4);
wb_keyboard_enable(TIME_STEP);
wb_inertial_unit_enable(Inertial_unit,TIME_STEP);
wb_lidar_enable(Lidar, TIME_STEP);
wb_gps_enable(GPS, TIME_STEP);
previous_point = [0,0];
current_point = [0,0];
previous_turning_angle = 0;
current_turning_angle = 0;
f = 0;
rng('default');
r1 = rand(random_delete_number,1);
while wb_robot_step(TIME_STEP) ~= -1
    %     grid = zeros(size_of_grid,size_of_grid);
    
    speed1 = zeros(4);
    f = f+1;
    key = wb_keyboard_get_key();
    switch(key)
        case {'w','W'}
            speed1 = speed_forward;
        case {'s','S'}
            speed1 = speed_backward;
        case {'a','A'}
            speed1 = speed_leftward;
        case {'d','D'}
            speed1 = speed_rightward;
        case {'q','Q'}
            speed1 = speed_leftCircle;
        case {'e','E'}
            speed1 = speed_rightCircle;
    end
    
    
    setspeed(motor1,motor2,motor3,motor4,speed1);
    inertial_unit_data = wb_inertial_unit_get_roll_pitch_yaw(Inertial_unit);
    
    %获取转向角，是一个浮点数
    turning_angle = inertial_unit_data(1);
    
    %获取雷达数据，是一个360个元素的数组，对应0-360到障碍物的距离
    range = wb_lidar_get_range_image(Lidar);
    
    %获取坐标位置，是一个浮点数三元组
    gps_data = wb_gps_get_values(GPS);
    
    previous_point = current_point;
    %将坐标位置映射到坐标
    current_point = [(gps_data(1)+5)*points_per_meter,(gps_data(2)+5)*points_per_meter];
    
    previous_turning_angle = current_turning_angle;%上一时刻的旋转角
    current_turning_angle = turning_angle;
    %获取雷达的数据点数
    horizontal_resolution = wb_lidar_get_horizontal_resolution(Lidar);
    %目标：把不能走的标上去，没标上去的都是能走的
    for i = 1:horizontal_resolution
        if range(i)==0||range(i)==inf
            continue;
        end
        location = (current_point-previous_point)*(i/horizontal_resolution) + previous_point;
        aim = (current_turning_angle-previous_turning_angle)*(double(i)/double(horizontal_resolution)) + previous_turning_angle;
        target_x = location(1) - (range(i) * sin(aim+(360-i)/360*2*pi))*points_per_meter;
        target_y = location(2) + (range(i) * cos(aim+(360-i)/360*2*pi))*points_per_meter;
        if int32(target_x)>size_of_grid||int32(target_y)>size_of_grid||int32(target_x)<1||int32(target_y)<1||size_of_grid - int32(target_y)<1||size_of_grid - int32(target_y)>1000
            continue;
        end
        %标注障碍物点
        avg = avg_grid(int32(target_x),int32(target_y));
        avg_grid(int32(target_x),int32(target_y)) = avg*(discount_rate-1) + 1;
        show(size_of_grid - int32(target_y)+1,int32(target_x)) = avg*(discount_rate-1) + 1;
        %标注非障碍物点
        dis = sqrt((location(1)-target_x)^2+(location(2)-target_y)^2);
        detl_x = int32(double(location(1)) + r1*double((target_x-location(1))));
        detl_y = int32(double(location(2)) + r1*double((target_y-location(2))));
        for k = (1:random_delete_number)
            ii = detl_x(k);
            j = detl_y(k);
            avg = avg_grid(ii,j);
            avg_grid(ii,j) = avg*discount_rate;
            show(int32(size_of_grid)+1 - j,ii) = avg*discount_rate;
        end
    end
    

    
    if mod(f,1000) == 0%显示已经探索好的图
        
        s = imfilter(show,filter);
        B = s(:,:)>leastI;%B为建好的栅格图
        se=strel('square',5);
        B=imclose(B,se);
        imshow(B);
        
    end
end
keyboard;
% cleanup code goes here: write data to files, etc.
