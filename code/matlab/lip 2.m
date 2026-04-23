if clientID > -1
    disp('Connected to CoppeliaSim');
    
    % بدء المحاكاة
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait);

    % الحصول على الـ handle الخاص بالكوادكوبتر
    [~, quadHandle] = sim.simxGetObjectHandle(clientID, 'Quadcopter_base', sim.simx_opmode_blocking);

    % تهيئة التخزين
    eulerAngles = [];
    timeStamps = [];

    % اجمع البيانات لمدة 20 ثانية مثلاً
    t0 = tic;
    while toc(t0) < 20
        [~, angles] = sim.simxGetObjectOrientation(clientID, quadHandle, -1, sim.simx_opmode_streaming);
        pause(0.05); % تأخير بسيط لجمع البيانات
        
        [~, angles] = sim.simxGetObjectOrientation(clientID, quadHandle, -1, sim.simx_opmode_buffer);
        
        if angles(1) ~= 0 || angles(2) ~= 0 || angles(3) ~= 0
            eulerAngles(end+1,:) = angles;  % angles = [Roll, Pitch, Yaw]
            timeStamps(end+1) = toc(t0);
        end
    end

    % إيقاف المحاكاة
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait);
    
    % فصل الاتصال
    sim.simxFinish(clientID);
else
    disp('Failed to connect to CoppeliaSim');
end

% رسم البيانات
figure;
plot(timeStamps, rad2deg(eulerAngles(:,1)), 'r', 'LineWidth', 1.5); hold on;
plot(timeStamps, rad2deg(eulerAngles(:,2)), 'g', 'LineWidth', 1.5);
plot(timeStamps, rad2deg(eulerAngles(:,3)), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Roll', 'Pitch', 'Yaw');
title('Euler Angles vs Time');
grid on;
