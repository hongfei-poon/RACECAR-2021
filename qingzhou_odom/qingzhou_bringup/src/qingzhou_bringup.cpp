#include "qingzhou_bringup/qingzhou_bringup.h"

long long LeftticksPerMeter = 0;    
long long rightticksPerMeter = 0;   
long long LeftticksPer2PI = 0;      
long long rightticksPer2PI = 0;     

// 构造函数，初始化
actuator::actuator(ros::NodeHandle handle) 
{ 
   servo_mid = 63;
   m_baudrate = 115200;             
   m_serialport = "/dev/ttyUSB0";   
   linearSpeed = 0;                 
   angularSpeed = 0;              
   batteryVoltage = 0;
   nav_status = NAVIGATION;
   goal_reached=true;
   light_info=0;

   line_info=0;
   lineTern = 0;
   line_kp=-0.35;
   line_kd=0;
   line_ki=0;
   line_vel=0.7;
   line_err=0;
   line_err_last=0;
   line_angle_out=0;
   

   vel1 = 0;
   ticksPerMeter = 0;               
   ticksPer2PI = 0;                 
   encoderLeft = 0;                 
   encoderRight = 0;               
   velDeltaTime = 0;                
   calibrate_lineSpeed = 0;         
   calibrate_angularSpeed = 0;
   err_rot = 0;
   err_rot_last=0;
   err_trans = 0;
   teb_min_pts = 7;
   follow_local_planner=false;
   controller_ang_kp=1.0;
   controller_ang_ki=0.0;
   controller_ang_kd=0.0;
   controller_c_rotation = 1.0;
   controller_c_translation = 0.5;
   controller_ang_factor_vel=1.0;
   controller_ang_out=0;
   
   controller_vel_kp=1.0;
   controller_vel_ki=0;
   controller_vel_out;
   v_teb_command=0;
   w_teb_command=0;
   max_vel=0.7;
   

//    v1 = 0;// 备用
//    w1 = 0;

//    v2 = 0;//测试专用
//    w2 = 0;

   angle2 = 0;                //测试专用
   direction2 = 0;
   vel2 = 0;



   
   test_array.data.resize(10);
   strcpy(socket_cmd,"pause");
   memset(&moveBaseControl,0,sizeof(sMartcarControl));
   
   handle.param("mcubaudrate",m_baudrate,m_baudrate);                                  
   handle.param("mcuserialport",m_serialport,std::string("/dev/ttyUSB0"));              
   handle.param("servo_mid",servo_mid,servo_mid);
   handle.param("calibrate_lineSpeed",calibrate_lineSpeed,calibrate_lineSpeed);         
   handle.param("calibrate_angularSpeed",calibrate_angularSpeed,calibrate_angularSpeed);
   handle.param("ticksPerMeter",ticksPerMeter,ticksPerMeter);                           
   handle.param("ticksPer2PI",ticksPer2PI,ticksPer2PI); 
   handle.param("lineTern", lineTern, lineTern);
   handle.param("vel1", vel1, vel1);
   handle.param("line_kp",line_kp,line_kp);
   handle.param("line_kd",line_kd,line_kd);
   handle.param("line_ki",line_ki,line_ki);
   handle.param("line_vel",line_vel,line_vel);
   handle.param("max_vel",max_vel,max_vel);
   
   handle.param("teb_min_pts", teb_min_pts, teb_min_pts);
   handle.param("controller_ang_kp", controller_ang_kp, controller_ang_kp);
   handle.param("controller_ang_kd", controller_ang_kd, controller_ang_kd);
   handle.param("controller_ang_ki", controller_ang_ki, controller_ang_ki);
   handle.param("controller_ang_factor_vel", controller_ang_factor_vel, controller_ang_factor_vel);
   handle.param("controller_c_rotation",controller_c_rotation,controller_c_rotation); 
   handle.param("controller_c_translation",controller_c_translation,controller_c_translation);       
   handle.param("controller_vel_kp", controller_vel_kp, controller_vel_kp);
   handle.param("controller_vel_ki", controller_vel_ki, controller_vel_ki);
   

//    handle.param("v1", v1, v1);  //备用
//    handle.param("w1", w1, w1);

//    handle.param("v2", v2, v2);    //测试专用
//    handle.param("w2", w2, w2);

   handle.param("angle2", angle2, angle2);             //测试专用
   handle.param("direction2", direction2, direction2);
   handle.param("vel2", vel2, vel2);


    try{ 
      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize start!"<<std::endl;              
      ser.setPort(m_serialport.c_str());                                               
      ser.setBaudrate(m_baudrate);                                                    
      serial::Timeout to = serial::Timeout::simpleTimeout(30);                      
      ser.setTimeout(to);                                                            
      ser.open();                                                                     
    }
    catch (serial::IOException& e){
      std::cout<<"[qingzhou_actuator-->]"<<"Unable to open port!"<<std::endl;                
    }
    if(ser.isOpen()){
      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize successfully!"<<std::endl;       
    }
    else{
      std::cout<<"[qingzhou_actuator-->]"<<"Serial port failed!"<<std::endl;                  
    } 
	
    sub_move_base = handle.subscribe("cmd_vel",1,&actuator::callback_move_base,this);
	sub_light_info = handle.subscribe("light_control", 5, &actuator::light_callback, this);
    sub_localpath = handle.subscribe("/move_base/TebLocalPlannerROS/local_plan",1,&actuator::localpath_callback,this);
    sub_nav_status = handle.subscribe("/move_base/TebLocalPlannerROS/GOAL_OR_NOT",1,&actuator::movebase_fb_callbcack,this);

    // cout << "sub_ok" << endl;
	// cout << light_info << endl;
	sub_line_info = handle.subscribe("line_number_direction", 5, &actuator::line_callback, this);
    sub_socket = handle.subscribe("/dispatcher/cmd",5,&actuator::socket_callback,this); 
    

    pub_imu = handle.advertise<sensor_msgs::Imu>("raw", 5);	                                                 
    pub_mag = handle.advertise<sensor_msgs::MagneticField>("imu/mag", 5);                                    
    pub_odom = handle.advertise<nav_msgs::Odometry>("odom", 5);                                               
    pub_battery = handle.advertise<std_msgs::Float32>("battery",10);                                          
    pub_targetpt = handle.advertise<geometry_msgs::PoseStamped>("target_point",1);
    pub_test = handle.advertise<std_msgs::Float32MultiArray>("bringup/test",1);
    timer_teb_control = handle.createTimer(ros::Duration(0.05),&actuator::teb_control_callback,this);
}

//析构函数
actuator::~actuator() 
{
     
}

void actuator::callback_move_base(const geometry_msgs::Twist::ConstPtr &msg) //对应cmd_vel话题，对应geometry_msgs/Twist消息
{
   memset(&moveBaseControl,0,sizeof(sMartcarControl));                       //清零movebase数据存储区
   v_teb_command = msg->linear.x;                                                  //move_base算得的线速度
   w_teb_command = msg->angular.z;                                               //move_base算得的角速度



}
  

void actuator::localpath_callback(const nav_msgs::Path::ConstPtr& msg)
{

    int num_of_pts=0;
    int target_index=0;
    //ROS_INFO("HEAR PATH,%d",num_of_pts);
    //ROS_INFO("MIN PTS,%d",teb_min_pts);
    teb_path=*msg;
    num_of_pts=teb_path.poses.size();
    if(num_of_pts==0)
    {
        ROS_INFO("ARRIVED");
        return;
    }
    else
    {
        vel_trans.resize(num_of_pts);
        vel_rot.resize(num_of_pts);
        for(int i=0;i<num_of_pts-1;i++)
        {
            
            
            float dt=teb_path.poses[i+1].header.stamp.toSec()-teb_path.poses[i].header.stamp.toSec();
            float yaw1=atan2(teb_path.poses[i+1].pose.orientation.z,teb_path.poses[i+1].pose.orientation.w);
            float yaw0=atan2(teb_path.poses[i].pose.orientation.z,teb_path.poses[i].pose.orientation.w);
            vel_rot[i]=(yaw1-yaw0)/dt;
            
            float x0=teb_path.poses[i].pose.position.x; float y0=teb_path.poses[i].pose.position.y;
            float x1=teb_path.poses[i+1].pose.position.x; float y1=teb_path.poses[i+1].pose.position.y;
            float dx=x0-x1;float dy=y0-y1;
            float dist=sqrt(dx*dx+dy*dy);
            vel_trans[i]=dist/dt;
        }
    }

    
    if(num_of_pts<teb_min_pts) 
    {
        follow_local_planner=true;
        target_index=num_of_pts-1;
        ROS_INFO("GOAL APPROACHING");
    }
    else
    {
        target_index=teb_min_pts-1;
        follow_local_planner=false;

    }
    try
    {
        tf_listener.transformPose("base_link",ros::Time(0),teb_path.poses[target_index],"odom",pose_target);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
}

void actuator::teb_control_callback(const ros::TimerEvent&)
{
    //ROS_INFO("TIMERCB");
    int num_of_pts=teb_path.poses.size();

    if(goal_reached)
    {
        pose_target.pose.position.x=0;
        pose_target.pose.position.y=0;
        pose_target.pose.orientation.x=0;
        pose_target.pose.orientation.y=0;
        pose_target.pose.orientation.z=0;
        pose_target.pose.orientation.w=1;
    }
    

    float dx=pose_target.pose.position.x;
    float dy=pose_target.pose.position.y;
    float diff_rot = 0;
    float diff_trans = 0;
    //ROS_INFO("TARGET:(%f,%f)",dx,dy);
    if(diff_rot<-3.13)
    {
        diff_rot=-3.13;

    }
    else;
    if(diff_rot>3.13)
    {
        diff_rot=3.13;
    }
    else;
    if(num_of_pts==0)
    {
        //ROS_INFO("000");
        diff_rot=0;
        diff_trans=0;
        dx=0;
        dy=0;
    }
    else
    {
        if(follow_local_planner==false)
        {
            
            //ROS_INFO("FAR");

            int sign=1;
            diff_rot = atan2(dy,dx);
            diff_trans = sqrt(dx * dx + dy * dy);
            
            
            if(dx>0) 
            {
                sign=1;
            }
            else if(dx==0)
            { 
                sign=0;
            }
            else 
            {
                //ROS_INFO("ROTATION=%f",diff_rot);
                sign = -1;
                if(diff_rot>0)
                {
                    diff_rot-=3.1415926535;

                }   
                else if(diff_rot==0)
                {
                    diff_rot=diff_rot;
                }
                else
                {
                    diff_rot+=3.1415926535;
                }
            }
            err_trans = sign*diff_trans;
            err_rot = sign*diff_rot;

            if(err_rot<-3.13)
            {
                err_rot=-3.13;

            }
            if(err_rot>3.13)
            {
                err_rot=3.13;
            }
            
            //ROS_INFO("FAR,dx=%f,dy=%f,dyaw=%f,err_rot=%f,sign=%d",dx,dy,diff_rot,err_rot,sign);
            //ROS_INFO("ERR_TRANS=%f,ERR_ROT=%f",err_trans,err_rot);

        }
        else
        {
            diff_rot = 2*atan2(pose_target.pose.orientation.z,pose_target.pose.orientation.w);
            diff_trans = sqrt(dx * dx + dy * dy);
            int sign = 1;

            if(dx > 0) sign = 1;
            else if(dx == 0) sign = 0;
            else sign = -1;
            err_trans = controller_c_translation * dx + sign * fabs(controller_c_rotation * diff_rot);
            err_rot = sign * diff_rot;           
            //ROS_INFO("NEAR,err_trans=%f,err_rot=%f,dx=%f,dy=%f",err_trans,err_rot,dx,dy);
            test_array.data[0]=err_rot;
            test_array.data[1]=err_trans;
            test_array.data[3]=moveBaseControl.TargetSpeed;            
        }



        //pose_target.pose.orientation.w=cos(err_rot);
        //pose_target.pose.orientation.z=sin(err_rot);
        //ROS_INFO("PUBLISH");
        pub_targetpt.publish(pose_target);
    }

    if(nav_status==NAVIGATION)
    {
        if(num_of_pts>0)
        {
            controller_ang_out=err_rot*controller_ang_kp+(err_rot-err_rot_last)*controller_ang_kd/20.0;
            controller_ang_out=controller_ang_out/(fabs(car_odom.twist.twist.linear.x)*controller_ang_factor_vel+0.3);
            err_rot_last=err_rot;
            controller_vel_out=controller_vel_kp*err_trans/20.0;
            if(controller_vel_out>max_vel)
            {
                moveBaseControl.TargetSpeed=max_vel;
            }

            moveBaseControl.TargetSpeed = controller_vel_out*32/0.43;                        //计算目标线速度
            moveBaseControl.TargetAngle = round(controller_ang_out*57.3);
            moveBaseControl.TargetAngle+=servo_mid;                                          //stm32 program has subtract 60

        }
        else
        {
            //ROS_INFO("NUMPTS=0");
            moveBaseControl.TargetSpeed=0;
            moveBaseControl.TargetAngle=servo_mid;

        }
        if(light_info==1)
        {
            ROS_WARN("RED LIGHT DETECTED,%f",light_info);
            moveBaseControl.TargetAngle = servo_mid;
            moveBaseControl.TargetSpeed = 0;
        }
        else
        {

        }
    }
    else
    {
        line_err=-line_info;
        line_angle_out=line_kp*line_err+line_kd*(line_err-line_err_last)/20.0;
        //ROS_INFO("VISUAL GUIDING");
        line_err_last=line_err;

        moveBaseControl.TargetAngle = line_angle_out;
        moveBaseControl.TargetAngle += servo_mid;
        moveBaseControl.TargetSpeed = line_vel*32/0.43;         
    }

    pub_test.publish(test_array);
    sendCarInfoKernel();   
}
void actuator::run()
{
    int run_rate = 100;              
    ros::Rate rate(run_rate);      

    double x = 0.0;                 //x坐标                       
    double y = 0.0;                 //y坐标
    double th = 0.0;

    ros::Time current_time, last_time;                    
	
    while(ros::ok()){
    ros::spinOnce();                                  
	current_time = ros::Time::now();                  //获得当前时间
	velDeltaTime = (current_time - last_time).toSec();//转换成秒
	last_time = ros::Time::now();                    
	



    recvCarInfoKernel();                              //接收stm32发来的数据
	pub_9250();                                       //发布imu数据
		
	currentBattery.data = batteryVoltage;            
	pub_battery.publish(currentBattery);             

    #if 1
	if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0;
	if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        //encoderLeft = -encoderLeft;
	encoderRight = -encoderRight;                          

	detEncode = (encoderLeft + encoderRight)/2;            
	detdistance = detEncode/ticksPerMeter;                 
	detth = (encoderRight - encoderLeft)*2*PI/ticksPer2PI; //计算当前角度,通过标定获得ticksPer2PI

	linearSpeed = detdistance/velDeltaTime;                
	angularSpeed = detth/velDeltaTime;                     
		
	if(detdistance != 0){
	    x += detdistance * cos(th);                        //x坐标
	    y += detdistance * sin(th);                        //y坐标
	}
	if(detth != 0){
	    th += detth;                                       //总角度
	}
       
	if(calibrate_lineSpeed == 1){
		printf("x=%.2f,y=%.2f,th=%.2f,linearSpeed=%.2f,,detEncode=%.2f,LeftticksPerMeter = %lld,rightticksPerMeter = %lld,batteryVoltage = %.2f\n",x,y,th,linearSpeed,detEncode,LeftticksPerMeter,rightticksPerMeter,batteryVoltage);
	}






	//send command to stm32
	                                                  
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); 

	nav_msgs::Odometry odom;                               //创建nav_msgs::Odometry类型的消息odom
	odom.header.stamp = current_time;                     
	odom.header.frame_id = "odom";                         
 	odom.child_frame_id = "base_link";                     

	//set the position
	odom.pose.pose.position.x = x;                       
	odom.pose.pose.position.y = y;                        
	odom.pose.pose.position.z = 0.0;                       
	odom.pose.pose.orientation = odom_quat;               
	
	odom.twist.twist.linear.x = linearSpeed;               //线速度
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = angularSpeed;             //角速度
	if(encoderLeft == 0 && encoderRight == 0){
	    odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,       
				                0, 1e-3, 1e-9, 0, 0, 0,
				                0, 0, 1e6, 0, 0, 0,
				                0, 0, 0, 1e6, 0, 0,
				                0, 0, 0, 0, 1e6, 0,
				                0, 0, 0, 0, 0, 1e-9};

		odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,      
						        0, 1e-3, 1e-9, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e-9};
		}
	else{
		odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,       
						        0, 1e-3, 0, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e3};

		odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,      
						        0, 1e-3, 0, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e3};
		}
	pub_odom.publish(odom);                                                            
	car_odom=odom;
#endif
    rate.sleep();
    }
}

//short int actuator::traffic_light(short int vel)
//{
//	ros::NodeHandle handle;
//	sub_light_info = handle.subscribe("light_contorl", 5,&actuator::light_callback,this);
//	ros::spin();
//	return vel;
//}

void actuator::light_callback(const std_msgs::Float32::ConstPtr& msg)
{
	light_info = msg->data;
	cout <<"light_info=" << light_info << endl;
}

void actuator::line_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size()!=2)
    {
        return;
    }
    else
    {
        if(msg->data[0]==0)
        {
            nav_status=NAVIGATION;
        }
        else
        {
            nav_status=VISUAL_CONTROL;
            line_info = msg->data[1];
            cout << "line_info=" << line_info << endl;
            cout << "-----------------------------------------"<<endl;
        }
    }
}

void actuator::socket_callback(const std_msgs::String::ConstPtr& msg)
{
    strcpy(socket_cmd,msg->data.c_str());
    cout<<"msg="<<socket_cmd<<endl;
}

void actuator::movebase_fb_callbcack(const std_msgs::Int16::ConstPtr& msg)
{
    //ROS_INFO("STATUS=%d",msg->status.status);
    if(msg->data==0)
    {
        goal_reached=false;

    }
    else
    {
        ROS_INFO("GOAL REACHED!");
        goal_reached=true;
    }
}
//发送小车数据到下位机
void actuator::sendCarInfoKernel()
{
	//short int vel;
    short int vel_t=(short)moveBaseControl.TargetSpeed;
	int angle1 = (int)moveBaseControl.TargetAngle;
	int direction = (int)moveBaseControl.TargetAngleDir;
    //ROS_WARN("SENDING");
    


    //上位机，停车和运行指令
    
    if(!strcmp("running",socket_cmd))
    {
        vel_t = vel_t;
        //ROS_ERROR("RUN");
    }

	else if(!strcmp("pausing",socket_cmd))  //判断红绿灯
    {
        vel_t = 0;
        angle1 = servo_mid;
        ROS_ERROR("STOP");
        // ros::Duration(5).sleep();    //休眠5秒
        // printf()
    }
	else
    {
        vel_t = vel_t;
    }
		
    // if(strcmp(socket_cmd,"pause"))  //socket停车命令
    // {
    //     vel_t=0;
    // }
    // else
    // {
    //     vel_t=vel_t;
    // }
	//if (line_info==0)    //跟随车道线
	//{
	//	angle1 = angle1;
	//	direction = direction;
	//}
	//else
	//{
	//	angle1 = lineTern;
	//	direction = 0;
	  //vel_t = vel1 ;
	//}
	
//direction = direction2;      //测试专用
    // angle1 = angle2 * line_info + 63;
	//angle1 = 63;
     //vel_t = vel2;

	//vel_t=10;
	//actuator::traffic_light(vel_t);
	//cout<<"vel_t="<<vel_t<<endl;

	// printf("angle1 in line =[%i] \n",angle1);
	// printf("direction=[%i]\n",direction);
	// printf("vel_t in line =[%i] \n",vel_t);


    unsigned char buf[23] = {0};
    buf[0] = 0xa5;	                                        
    buf[1] = 0x5a;	                                        
    buf[2] = 0x06;	                                        

    buf[3] = (int)direction;	    //targetangleDirection 0-->go straight,0x10-->turn left,0x20-->turn right (not used)
    buf[4] = (int)abs(angle1);	    //targetangle
    //buf[5] = (int)abs(moveBaseControl.TargetSpeed);	    //targetSpeed
    //buf[6] = (int)moveBaseControl.TargetModeSelect;	    //0-->person control,1-->auto control (not used)
    buf[5] = (unsigned char)((vel_t>>8)&0x0ff);
    buf[6] = (unsigned char)((vel_t)&0x0ff);
    // float test_vel=(float)((short)(buf[5])<<8+(short)buf[6]);
    //test_vel=(test_vel<<8)+buf[6];
    // cout<<"vel="<<test_vel<<endl;
    buf[7] = (int)moveBaseControl.TargetShiftPosition;  //targetshiftposition  0-->P stop;1-->R;2-->D. (not used)
  
    buf[8] = 0;		                                        
    unsigned char sum = 0;
    for(int i = 2; i < 19; ++i)                             
        sum += buf[i];
    buf[9] = (unsigned char)(sum);                      
    size_t writesize = ser.write(buf,10);
}

//接收下位机发送来的数据
void actuator::recvCarInfoKernel()
{
    std::string recvstr;                                             
    unsigned char tempdata,lenrecv;                                  
    unsigned char count,last_data,last_last_data,last_last_last_data;
    unsigned char str[100];                                          
    bool recvflag = false;                                          
    bool recvd_flag = false;                                         
    memset(&str,0,sizeof(str));                                      
    ros::Time begin_time = ros::Time::now();                         
    double clustering_time = 0;                                      

    while(1){
	    clustering_time = (ros::Time::now () - begin_time).toSec (); //计算时间差，转换成秒
	    if(clustering_time > 1){                                    
	        recvd_flag = false;                                      
	        break;                                                   
	    }
		
	    recvstr = ser.read(1);                                      		
	    if((int)recvstr.size() != 1)                                 
	        continue;                                               
		
	    tempdata = recvstr[0];                                       
	    if(last_last_last_data == 0xa5 && last_last_data == 0x5a){   
	        lenrecv = last_data;                                     
	        recvflag = true;                                         
	        count = 0;                                               
	    }
	    if(recvflag){                                               
	        str[count] = tempdata;                                   
	        count++;                                                
	        if(count == lenrecv){                                   
		        recvflag = false;                                    
		        recvd_flag = true;                                   
		    break;                                                  
	        }
	    }
	    last_last_last_data = last_last_data;                        
	    last_last_data = last_data;                                  
	    last_data = tempdata;                                        
    }

    if(recvd_flag){                                                  //数据解析，接收到的数据转存
	    memcpy(&encoderLeft,str,4);                                  
	    memcpy(&encoderRight,str+4,4);                              
	    memcpy(&batteryVoltage,str+8,4);                           
 	
	    memcpy(&tempaccelX,str+12,2);                               
	    memcpy(&tempaccelY,str+14,2);                               
		memcpy(&tempaccelZ,str+16,2);                                
		
		memcpy(&tempgyroX,str+18,2);                                 
		memcpy(&tempgyroY,str+20,2);                                
		memcpy(&tempgyroZ,str+22,2);                                 
		
		memcpy(&tempmagX,str+24,2);                                 
		memcpy(&tempmagY,str+26,2);                                
		memcpy(&tempmagZ,str+28,2);                                

		accelX = (float)tempaccelX/2048*9.8;                         //线加速度处理	
		accelY = (float)tempaccelY/2048*9.8;
		accelZ = (float)tempaccelZ/2048*9.8;

		
		//gyroX = (float)tempgyroX/16.4/57.3;                          //角速度处理
		//gyroY = (float)tempgyroY/16.4/57.3;
		gyroX=0;
        gyroY=0;
        gyroZ = (float)tempgyroZ/16.4/57.3;
		
		magX = (float)tempmagX*0.14;                                 //磁力计处理
		magY = (float)tempmagY*0.14;
		//magZ = (float)tempmagZ*0.14;
        magZ =0 ;
        if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0; //判断编码器脉冲数是否在正确范围
        if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        LeftticksPerMeter += encoderLeft;                            //获得左轮总脉冲数
        rightticksPerMeter += encoderRight;                          //获得右轮总脉冲数
    }
}

//发布imu函数
void actuator::pub_9250(){                         
    sensor_msgs::Imu imuMsg;                       
    sensor_msgs::MagneticField magMsg;           
	
    ros::Time current_time = ros::Time::now();    
	         
    imuMsg.header.stamp = current_time;            
    imuMsg.header.frame_id = "imu_link";          
    imuMsg.angular_velocity.x = gyroX;             
    imuMsg.angular_velocity.y = gyroY;             
    imuMsg.angular_velocity.z = gyroZ;             
    imuMsg.angular_velocity_covariance = {         
      0.6,0.0,0.0,
      0.0,0.6,0.0,
      0.0,0.0,0.6
    };
    
    imuMsg.linear_acceleration.x = accelX;        
    imuMsg.linear_acceleration.y = accelY;         
    imuMsg.linear_acceleration.z = accelZ;        
    imuMsg.linear_acceleration_covariance = {      
      0.04,0.0,0.0,
      0.0,0.04,0.0,
      0.0,0.0,0.04
    };
    pub_imu.publish(imuMsg);                       //发布imuMsg 
     
    magMsg.header.stamp = current_time;            
    magMsg.header.frame_id = "base_link";          
    magMsg.magnetic_field.x = magX;                
    magMsg.magnetic_field.y = magY;                
    magMsg.magnetic_field.z = magZ;                
    magMsg.magnetic_field_covariance = {           
      0.0,0.0,0.0,
      0.0,0.0,0.0,
      0.0,0.0,0.0
    };
    pub_mag.publish(magMsg);                       //发布magMsg
}
