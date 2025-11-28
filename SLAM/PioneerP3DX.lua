function sysCall_init()
    robot = sim.getObject('.')
    leftMotor = sim.getObject("./leftMotor")
    rightMotor = sim.getObject("./rightMotor")
    laser = sim.getObject("./lidar")
    
    simROS2 = require('simROS2')
    
    -- Publicadores
    pub_scan = simROS2.createPublisher('/scan', 'sensor_msgs/msg/LaserScan')
    pub_odom = simROS2.createPublisher('/odom', 'nav_msgs/msg/Odometry')
    
    -- Suscriptor
    sub_vel = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'cmd_vel_callback')
    
    wheel_radius = 0.0975; axis_length = 0.381; v_l = 0; v_r = 0
end

function cmd_vel_callback(msg)
    local v_lin = msg.linear.x; local v_ang = msg.angular.z
    v_l = (v_lin - (axis_length * v_ang) / 2) / wheel_radius
    v_r = (v_lin + (axis_length * v_ang) / 2) / wheel_radius
end

function sysCall_sensing()
    local t = simROS2.getTime()
    
    -- 1. TF (Transformadas)
    local pos = sim.getObjectPosition(robot, -1)
    local orient = sim.getObjectQuaternion(robot, -1)
    
    local tf_odom = {
        header = { stamp = t, frame_id = 'odom' },
        child_frame_id = 'base_link',
        transform = {
            translation = { x = pos[1], y = pos[2], z = pos[3] },
            rotation = { x = orient[1], y = orient[2], z = orient[3], w = orient[4] }
        }
    }
    simROS2.sendTransform(tf_odom)
    
    -- 2. ODOMETRÍA
    local odom_msg = {
        header = { stamp = t, frame_id = 'odom' },
        child_frame_id = 'base_link',
        pose = { pose = { position = tf_odom.transform.translation, orientation = tf_odom.transform.rotation } }
    }
    simROS2.publish(pub_odom, odom_msg)

    -- 3. LÁSER (CORRECCIÓN MATEMÁTICA)
    local dataSignal = sim.getStringSignal("PioneerP3dxLidarData")
    if dataSignal then
        local ranges = sim.unpackFloatTable(dataSignal)
        if ranges and #ranges > 0 then
            -- Definimos el campo de visión (240 grados)
            local field_of_view = 240 * (math.pi / 180)
            
            -- Calculamos el incremento exacto
            -- Usamos (#ranges - 1) para distribuir los puntos perfectamente de principio a fin
            local angle_inc = field_of_view / (#ranges - 1)
            
            local min_angle = -field_of_view / 2
            
            -- TRUCO: Calculamos el ángulo máximo basándonos en los puntos que tenemos.
            -- Así ROS nunca se quejará de que faltan o sobran puntos.
            local max_angle = min_angle + (angle_inc * (#ranges - 1))
            
            local msg = {
                header = { stamp = t, frame_id = 'base_link' }, 
                angle_min = min_angle,
                angle_max = max_angle, -- Usamos el calculado
                angle_increment = angle_inc,
                time_increment = 0.0,
                scan_time = sim.getSimulationTimeStep(),
                range_min = 0.02, 
                range_max = 5.0, 
                ranges = ranges
            }
            simROS2.publish(pub_scan, msg)
        end
    end
end

function sysCall_actuation()
    sim.setJointTargetVelocity(leftMotor, v_l)
    sim.setJointTargetVelocity(rightMotor, v_r)
end