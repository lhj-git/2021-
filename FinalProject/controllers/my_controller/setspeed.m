function setspeed(motor1,motor2,motor3,motor4,speed)
%SETSPEED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    wb_motor_set_velocity(motor1,speed(1));
    wb_motor_set_velocity(motor2,speed(2));
    wb_motor_set_velocity(motor3,speed(3));
    wb_motor_set_velocity(motor4,speed(4));
end

