    
    
    cago_initial={1.565, 0.939, -0.056, 0.975, 1.738, 0.023, -0.034};  //{-1.1470 0.3418 -2.1480}
    // cago_reference1={2.228, 0.000,0.000,0.000, 0.000, 0.000, 0.000};  {-0.2391, 0.3734 ,-0.04949} left
    // cago_reference2={0.817, 0.000,0.000,0.000, 0.000, 0.000, 0.000}; {0.4615, 0.4492, 0.4492} right
    // cago_reference3={1.374, 0.598, 0.000,0.000, 0.000, 0.000, 0.000};  {0.1419, 0.3628, -0.2444} top
    move2={ 0.8776  , -0.9723  ,  0.2294  , -0.3972  ,  0.3162    ,     0     , 0.000}; //0.09184,0.5204,1.186 square starts here
    move3={ 1.0337 ,  -0.3533  , -0.3469  , -0.9676  ,  0.7491    ,     0 ,0.000}; //0.09184,0.398,0.08492
    move4={ 1.0337 ,  -0.3533  , -0.3469  , -0.9676  ,  0.7491    ,     0  ,0.000}; //0.2143,0.5204,1.154
    move5={  0.8776 ,  -0.9723 ,   0.2294  , -0.3972   , 0.3162     ,    0 ,    0,0.000};//-0.02041,-0.06122,-0.07396
    //move6={1.9253  ,  1.1397  ,  0.2383   , 1.0698  ,  1.9624  , -0.2498, -0.034}; //-1.429,-0.6122,-2.485
    //move7={1.5532  ,  1.0298  , -0.2537  ,  1.1005   ,1.8147  ,  0.2033,-0.034}; //-0.6122,0.2041,-2.188

    // % Set joint limits for the first joint

    // robot.links(1).qlim = [-pi/2, pi/2];

    // trajectory_msgs::msg::JointTrajectoryPoint::time_from_start

    // arm_1_joint: <limit effort="43.0" lower="0.0" upper="2.748893571891069" velocity="1.95"/>
    // arm_2_joint: <limit effort="43.0" lower="-1.5707963267948966" upper="1.090830782496456" velocity="1.95"/>
    // arm_3_joint: <limit effort="26" lower="-3.5342917352885173" upper="1.5707963267948966" velocity="2.35"/>
    // arm_4_joint: <limit effort="26" lower="-0.39269908169872414" upper="2.356194490192345" velocity="2.35"/>
    // arm_5_joint: <limit effort="3" lower="-2.0943951023931953" upper="2.0943951023931953" velocity="1.95"/>
    // arm_6_joint: <limit effort="6.6" lower="-1.413716694115407" upper="1.413716694115407" velocity="1.76"/>
    // arm_7_joint: <limit effort="6.6" lower="-2.0943951023931953" upper="2.0943951023931953" velocity="1.76"/>
    // arm_tool_joint: fixed

    //(reference) T6 = [0.9897   0.0234   0.1409  -1.1470; -0.1401 -0.0323  0.9896  0.3418;  0.0277 -0.9992  -0.0287  -3.1480;   0    0  0  1.0000]
    // T6 =[      
    //     0.9897    0.0234    0.1409   -1.1470;
    //    -0.1401   -0.0323    0.9896    0.3418;
    //     0.0277   -0.9992   -0.0287   -3.1480;
    //          0         0         0    1.0000]
    //ros2 action send_goal /play_motion2 play_motion2_msgs/action/PlayMotion2 "{motion_name: home, skip_planning: false}"
    trajectory_msgs::msg::JointTrajectoryPoint p1,p2,p3,p4,p5,p6,p7;
    
    p1.positions = cago_initial;
    p1.time_from_start = rclcpp::Duration::from_seconds(3.0);
    points.push_back(p1);

    p2.positions= move2;
    p2.time_from_start = rclcpp::Duration::from_seconds(6.0);
    points.push_back(p2);

    p3.positions=move3;
    p3.time_from_start=rclcpp::Duration::from_seconds(9.0);
    points.push_back(p3);

    // p4.positions=move4;
    // p4.time_from_start=rclcpp::Duration::from_seconds(8.0);
    // points.push_back(p4);

    // p5.positions=move5;
    // p5.time_from_start=rclcpp::Duration::from_seconds(10.0);
    // points.push_back(p5);

    p2.positions= move2;
    p2.time_from_start = rclcpp::Duration::from_seconds(12.0);
    points.push_back(p2);

    // p6.positions=move6;
    // p6.time_from_start = rclcpp::Duration::from_seconds(10.0);
    // points.push_back(p6);

    // p7.positions=move7;
    // p7.time_from_start = rclcpp::Duration::from_seconds(12.0);
    // points.push_back(p7);

    p1.positions = cago_initial;
    p1.time_from_start = rclcpp::Duration::from_seconds(14.0);
    points.push_back(p1);