#include "paramListener.h"
#define GPIO_PORT gpio256
#include <serial/serial.h> 
 
// Param_listener::Param_listener(rclcpp::Node::SharedPtr node("my listner node"))
Param_listener::Param_listener(const std::string &node_name):BFSMulti()
{ // constructor

            auto node = rclcpp::Node::make_shared(node_name);
            this->nh_ = node;
            RCLCPP_INFO(nh_->get_logger(),"in class constructor of Param_listener");

            //initialize camera 
            //set initial values

			listen_param(); 
    		initParamMap();
    		
			createDayFolder();

            cout<<"open system\n";

			if(!systemCreated)
            	system = System::GetInstance();

			systemCreated = true;

            RCLCPP_INFO(nh_->get_logger(),"Going into spin; let the callbacks do all the work");

            rclcpp::executors::MultiThreadedExecutor spinner(rclcpp::ExecutorOptions(), 4);
            // spinner.set_num_threads(4);
            spinner.add_node(nh_);
            spinner.spin();

}

Param_listener::~Param_listener()
{

    cout<<"disconnect cameras\n";

    cout<<"close system\n";

    system->ReleaseInstance(); 
}


void Param_listener::listen_param(){

	  cout<<"listen_param\n";
      // RCLCPP_INFO(nh_->get_logger(),"in listen_param of Param_listener");

	  string robot_namespace = "/robot";
	
	  // connect_sub = nh_->subscribe("connect", 1000, &Param_listener::connectCallback, this);
      connect_sub = nh_->create_subscription<std_msgs::msg::String>("connect", 1000, std::bind(&Param_listener::connectCallback, this, std::placeholders::_1));
      // RCLCPP_INFO(nh_->get_logger(), "Subscribed to /robot/connect topic");


      frame_rate_sub = nh_->create_subscription<std_msgs::msg::String>("frameRate", 1000, std::bind(&Param_listener::frameRateCallback, this, std::placeholders::_1));

	  capture_sub = nh_->create_subscription<std_msgs::msg::String>("capture", 1000, std::bind(&Param_listener::captureCallback, this, std::placeholders::_1));

	  cont_trigger_sub = nh_->create_subscription<std_msgs::msg::String>("contTrigger", 1000, std::bind(&Param_listener::contTriggerCallback, this, std::placeholders::_1));
  
      stop_cont_trigger_sub = nh_->create_subscription<std_msgs::msg::String>("stopContTrigger", 1000, std::bind(&Param_listener::stopContTriggerCallback, this, std::placeholders::_1));

      disconnect_sub = nh_->create_subscription<std_msgs::msg::String>("disconnect", 1000, std::bind(&Param_listener::disconnectCallback, this, std::placeholders::_1));
      
      exposureTime_sub = nh_->create_subscription<std_msgs::msg::String>("exposureTime", 1000, std::bind(&Param_listener::exposureTimeCallback, this, std::placeholders::_1));
      
      folderName_sub = nh_->create_subscription<std_msgs::msg::String>("folderName", 1000, std::bind(&Param_listener::folderNameCallback, this, std::placeholders::_1));
	
	  balanceWhiteAuto_sub = nh_->create_subscription<std_msgs::msg::String>("balanceWhiteAuto", 1000, std::bind(&Param_listener::balanceWhiteAutoCallback, this, std::placeholders::_1));
      
      start_preview_sub = nh_->create_subscription<std_msgs::msg::String>("startPreview", 1000, std::bind(&Param_listener::startPreviewCallback, this, std::placeholders::_1));

  	  stop_preview_sub = nh_->create_subscription<std_msgs::msg::String>("stopPreview", 1000, std::bind(&Param_listener::stopPreviewCallback, this, std::placeholders::_1));
}

void Param_listener::createDayFolder(){

	cout<<"create Data folder\n";
	
    std::filesystem::path dird(dataPath);

    if(std::filesystem::create_directory(dird)){

         cout<<"Directory created: "<< dataPath<<endl;
    }
    
    CurrentDayFolder = dataPath + getCurrentDateStr();

    const char* path = CurrentDayFolder.c_str();

    std::filesystem::path dir(path);

	cout<<"create folder\n";

    if(std::filesystem::create_directory(dir)){

        cout<<"Directory created: "<< CurrentDayFolder<<endl;
    }

	cout<<"created\n";
}

string Param_listener::getCurrentDateStr(){

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y-%m-%d", std::localtime(&now));
    return s;
}


void Param_listener::connectCallback(const std_msgs::msg::String::SharedPtr msg){

    //create folder for today

    // RCLCPP_INFO(nh_->get_logger(),"in class constructor of Param_listener");

    RCLCPP_INFO(nh_->get_logger(),"I heard: [%s]", msg->data.c_str());


	camList = system->GetCameras();

	unsigned int numCameras = camList.GetSize();

	cout << "Number of cameras detected: " << numCameras << endl << endl;
	
       // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "No enough cameras!" << endl;
		
		return;
    }

	serial::Serial ser; 

	serial::Timeout to = serial::Timeout::simpleTimeout(500); 

    ser.setTimeout(to);
    
    ser.setPort("/dev/ttyACM0"); 

    ser.setBaudrate(9600);
    if(!ser.isOpen())
        ser.open();
 
    ser.write("gpio set 0\r");
    ser.write("gpio set 1\r");
    ser.write("gpio set 2\r");
    ser.write("gpio set 3\r");
    ser.write("gpio set 4\r");
	for(int id = 0; id < numCameras; id++)
    {

		CameraPtr pCam = camList.GetByIndex(id);

	    // boost::shared_ptr<BFS_camera> pCamera(new BFS_camera(pCam));

        std::shared_ptr<BFS_camera> pCamera = std::make_shared<BFS_camera>(pCam);
        
        pCamera->cam_connect();

        string camID = pCamera->cameraID;

		cout<<"camID: "<<camID<<endl;

		pCameras->BFS_new(pCamera, paras_cams[camID]);

		cout<<"connect to camera "<<camID<<endl;
	}

	std::cout<<"Cameras connected: " << pCameras->nCams_ <<std::endl;
	
        //for preview function
    image_transport::ImageTransport it(nh_); 

    pub1 = it.advertise("camera1/image", 1);
    pub2 = it.advertise("camera2/image", 1);
    // pub3 = it.advertise("camera3/image", 1);
    // pub4 = it.advertise("camera4/image", 1);
    // pub5 = it.advertise("camera5/image", 1);
    // pub6 = it.advertise("camera6/image", 1);
    // pub7 = it.advertise("camera7/image", 1);
    // pub8 = it.advertise("camera8/image", 1);
    // pub9 = it.advertise("camera9/image", 1);
    // pub10 = it.advertise("camera10/image", 1);
    pubvector.push_back(pub1);
    pubvector.push_back(pub2);
    // pubvector.push_back(pub3);
    // pubvector.push_back(pub4);
    // pubvector.push_back(pub5);
    // pubvector.push_back(pub6);
    // pubvector.push_back(pub7);
    // pubvector.push_back(pub8);
    // pubvector.push_back(pub9);
    // pubvector.push_back(pub10);

}

void Param_listener::frameRateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(nh_->get_logger(),"I heard frame rate: [%s]", msg->data.c_str());

  if(params_rev["frameRate"].compare(msg->data.c_str()) !=0){
    
    params_rev["frameRate"]= msg->data.c_str();

	frame_rate = stod(msg->data.c_str());

  }

}

void Param_listener::captureCallback(const std_msgs::msg::String::SharedPtr msg){

	RCLCPP_INFO(nh_->get_logger(),"I heard: [%s]", msg->data.c_str());

	pCameras->initializeIds();
         
    pCameras->BFSM_start_capturing();  

}

void Param_listener::preciseSleep(double seconds){

    using namespace std;
    using namespace std::chrono;

    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    while (seconds > estimate) {
        auto start = high_resolution_clock::now();
        this_thread::sleep_for(milliseconds(1));
        auto end = high_resolution_clock::now();

        double observed = (end - start).count() / 1e9;
        seconds -= observed;

        ++count;
        double delta = observed - mean;
        mean += delta / count;
        m2   += delta * (observed - mean);
        double stddev = sqrt(m2 / (count - 1));
        estimate = mean + stddev;
    }

    // spin lock
    auto start = high_resolution_clock::now();
    while ((high_resolution_clock::now() - start).count() / 1e9 < seconds);
}


void Param_listener::contTriggerCallback(const std_msgs::msg::String::SharedPtr msg){

    RCLCPP_INFO(nh_->get_logger(),"I heard: [%s]", msg->data.c_str());

	string ts;

	int count = 0;

	double initial_count = cv::getTickCount();

	pCameras->cont_triggering_ = true;

   // sleep_time = int(1000/frame_rate);
    sleep_time = int(1);
    cout<<"frame rate: "<<frame_rate<<"sleep time: "<<sleep_time<<endl;
    
    while(pCameras->cont_triggering_){
    
    	count++;
    	
		ts = pCameras->BFSM_getCurrentTimeStr();
		// rostimeSec = rclcpp::Time::now().toSec();
        rostimeSec = nh_->get_clock()->now().seconds();
		str = to_string(rostimeSec);
    	int r_tg = pCameras->BFSM_trigger(ts, str);

    	if(r_tg!=0)
            std::cout<<"Trigger Failed"<< std::endl;
            
        //std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time)); //60 for usb 3; 30 for gigE
        preciseSleep(0.25);

    }  

	float dura = ((double)cv::getTickCount() - initial_count)/cv::getTickFrequency();
    std::cout<<"stop count "<<dura<<endl;
	cout<<"FPS: "<<count/dura<<endl;
}

void Param_listener::stopContTriggerCallback(const std_msgs::msg::String::SharedPtr msg){

  	RCLCPP_INFO(nh_->get_logger(),"I heard: [%s]", msg->data.c_str());

    pCameras->BFSM_stop_trigger();

	//pCameras->cont_triggering_ = false;

}


void Param_listener::disconnectCallback(const std_msgs::msg::String::SharedPtr msg){  

    RCLCPP_INFO(nh_->get_logger(),"I heard: [%s]", msg->data.c_str());

    pCameras->BFSM_disconnect();

    camList.Clear();

}

void Param_listener::initParamMap(){

    cout<<"initParamMap\n";

    params_rev["exposureTime"] = "1000";

}

void Param_listener::exposureTimeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(nh_->get_logger(),"I heard exposureTime: [%s]", msg->data.c_str());

  if(params_rev["exposureTime"].compare(msg->data.c_str()) !=0){
    
    cout<<"before setting\n";

    pCameras->BFSM_set_exposure(stof(msg->data.c_str())); // 

    params_rev["exposureTime"]= msg->data.c_str();

  }
  
}

void Param_listener::folderNameCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(nh_->get_logger(),"I heard folderName: [%s]", msg->data.c_str());

	//write to txt
  myfile.open("/home/ur5/prem/camera_data/foldernames.txt", std::ios_base::app);
  myfile<<msg->data.c_str();
  myfile<<"\n";
  myfile.close();
  
  if(params_rev["folderName"].compare(msg->data.c_str()) !=0){
    
    cout<<"create a folder\n";

    //pCameras->BFSM_set_exposure(stof(msg->data.c_str())); // 

    params_rev["folderName"]= msg->data.c_str();
    
    //CurrentRowFolder = dataPath + getCurrentDateStr() + "/"+msg->data.c_str()+"/";
    
    CurrentRowFolder = dataPath+msg->data.c_str();


    cout<<"CurrentRowFolder: "<<CurrentRowFolder<<endl;
    
    const char* path = CurrentRowFolder.c_str();

    std::filesystem::path dir(path);

	cout<<"create folder\n";

    if(std::filesystem::create_directory(dir)){

        cout<<"Directory created: "<< CurrentRowFolder<<endl;
    }

	//cout<<"created \n";
	
	pCameras->BFSM_set_folder(CurrentRowFolder);

  }
  
}
void Param_listener::balanceWhiteAutoCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(nh_->get_logger(),"I heard balanceWhiteAuto: [%s]", msg->data.c_str());

  if(params_rev["balanceWhiteAuto"].compare(msg->data.c_str()) !=0){
    
    cout<<"before setting\n";

    pCameras->BFSM_set_balanceWhiteAuto(msg->data.c_str()); 

    params_rev["balanceWhiteAuto"]= msg->data.c_str();

  }
  
}


void Param_listener::startPreviewCallback(const std_msgs::msg::String::SharedPtr msg){

    RCLCPP_INFO(nh_->get_logger(),"I heard: [%s]", msg->data.c_str());

    if(pCameras->cameras_.size()>0){

        cout<<"camera size: "<<pCameras->cameras_.size()<<endl;

        pCameras->BFS_if_preview = true;

        cout<<"set if_preview to be true\n";

        for(ptrCamera pCamera_ : pCameras->cameras_)
        { 
            pCamera_->if_preview = true;
        } 

        cout<<"add to thread\n";

        //preview_thread_added
        _threadWorker = std::thread(&Param_listener::preview, this);

    }

}

void Param_listener::stopPreviewCallback(const std_msgs::msg::String::SharedPtr msg){

        RCLCPP_INFO(nh_->get_logger(),"I heard: [%s]", msg->data.c_str());

        pCameras->BFS_if_preview = false;

        for(ptrCamera pCamera_ : pCameras->cameras_)
        { 
            pCamera_->if_preview = false;
        } 

        _threadWorker.join(); 
 
}

void Param_listener::preview(){
    
        cout<<"publish image\n";

        while(pCameras->BFS_if_preview){
		
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // sensor_msgs::ImagePtr msg;
            sensor_msgs::msg::Image::SharedPtr msg;

            cam_count = 0;
            
            for(ptrCamera pCamera_ : pCameras->cameras_){
            
		        if((pCamera_->publishImage).empty()){
					cout<<"publishImage is empty\n";
		            continue;
		         }
                //cout<<"publish one image\n";
		        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", pCamera_->publishImage).toImageMsg();
		        pub = pubvector.at(cam_count);
		        pub.publish(msg);
		        cam_count++;
             }

        }

}

