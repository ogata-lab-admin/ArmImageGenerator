// -*- C++ -*-
/*!
 * @file  ArmImageGenerator.cpp
 * @brief Arm Image Generator RT Component
 * @date $Date$
 *
 * $Id$
 */

#include "ArmImageGenerator.h"

#include <iomanip>
//#include <fstream>
#include <ctime>


#define _USE_MATH_DEFINES
#include <math.h>

#ifdef WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif


// Module specification
// <rtc-template block="module_spec">
static const char* armimagegenerator_spec[] =
  {
    "implementation_id", "ArmImageGenerator",
    "type_name",         "ArmImageGenerator",
    "description",       "Arm Image Generator RT Component",
    "version",           "1.0.0",
    "vendor",            "ogata_lab",
    "category",          "Experimental",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "1",
    "conf.default.j0max", "1.57076",
    "conf.default.j1max", "1.57076",
    "conf.default.j0min", "-1.57076",
    "conf.default.j1min", "-1.57076",
    "conf.default.j0step", "0.157076",
    "conf.default.j1step", "0.157076",
	"conf.default.wait_interval", "1.0",
    "conf.default.camera_wait_time", "3.0",
    "conf.default.gripper_close_ratio", "0.1",
    "conf.default.camera_jointPos0", "1.57076",
    "conf.default.camera_jointPos1", "0",
    "conf.default.camera_jointPos2", "1.57076",
    "conf.default.camera_jointPos3", "0",
    "conf.default.camera_jointPos4", "1.57076",
    "conf.default.camera_jointPos5", "0",
    "conf.default.initial_jointPos0", "0",
    "conf.default.initial_jointPos1", "0",
    "conf.default.initial_jointPos2", "1.57076",
    "conf.default.initial_jointPos3", "0",
    "conf.default.initial_jointPos4", "1.57076",
    "conf.default.initial_jointPos5", "0",    
    
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.j0max", "text",
    "conf.__widget__.j1max", "text",
    "conf.__widget__.j0min", "text",
    "conf.__widget__.j1min", "text",
    "conf.__widget__.j0step", "text",
    "conf.__widget__.j1step", "text",

    "conf.__widget__.camera_jointPos0", "text",
    "conf.__widget__.camera_jointPos1", "text",
    "conf.__widget__.camera_jointPos2", "text",
    "conf.__widget__.camera_jointPos3", "text",
    "conf.__widget__.camera_jointPos4", "text",
    "conf.__widget__.camera_jointPos5", "text",

    "conf.__widget__.initial_jointPos0", "text",
    "conf.__widget__.initial_jointPos1", "text",
    "conf.__widget__.initial_jointPos2", "text",
    "conf.__widget__.initial_jointPos3", "text",
    "conf.__widget__.initial_jointPos4", "text",
    "conf.__widget__.initial_jointPos5", "text",

    
    "conf.__widget__.gripper_close_ratio", "slider.0.1",
    // Constraints
    "conf.__constraints__.gripper_close_ratio", "0.0<=x<=1.0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ArmImageGenerator::ArmImageGenerator(RTC::Manager* manager)
// <rtc-template block="initializer">
: RTC::DataFlowComponentBase(manager),
m_cameraIn("camera", m_camera),
m_manipCommonPort("manipCommon"),
m_manipMiddlePort("manipMiddle")

// </rtc-template>
, m_jointPos(new JARA_ARM::JointPos())
{
}

/*!
 * @brief destructor
 */
ArmImageGenerator::~ArmImageGenerator()
{
}



RTC::ReturnCode_t ArmImageGenerator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("camera", m_cameraIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_manipCommonPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_manipCommon);
  m_manipMiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_manipMiddle);
  
  // Set CORBA Service Ports
  addPort(m_manipCommonPort);
  addPort(m_manipMiddlePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  bindParameter("j0max", m_j0max, "1.57076");
  bindParameter("j1max", m_j1max, "1.57076");
  bindParameter("j0min", m_j0min, "-1.57076");
  bindParameter("j1min", m_j1min, "-1.57076");
  bindParameter("j0step", m_j0step, "0.157076");
  bindParameter("j1step", m_j1step, "0.157076");

  // the interval until the next movement
  bindParameter("wait_interval", m_wait_interval, "0.2");
  // the wating time for taking a picuture 
  bindParameter("camera_wait_time", m_camera_wait_time, "3.0");
  // the ratio of closed gripper
  bindParameter("gripper_close_ratio", m_gripper_close_ratio, "0.1");

  // the position for taking a picture 
  bindParameter("camera_jointPos0", m_camera_jointPos0, "1.57076");
  bindParameter("camera_jointPos1", m_camera_jointPos1, "0");
  bindParameter("camera_jointPos2", m_camera_jointPos2, "1.57076");
  bindParameter("camera_jointPos3", m_camera_jointPos3, "0");
  bindParameter("camera_jointPos4", m_camera_jointPos4, "1.57076");
  bindParameter("camera_jointPos5", m_camera_jointPos5, "0");

  // the initial position
  bindParameter("initial_jointPos0", m_initial_jointPos0, "0");
  bindParameter("initial_jointPos1", m_initial_jointPos1, "0");
  bindParameter("initial_jointPos2", m_initial_jointPos2, "1.57076");
  bindParameter("initial_jointPos3", m_initial_jointPos3, "0");
  bindParameter("initial_jointPos4", m_initial_jointPos4, "1.57076");
  bindParameter("initial_jointPos5", m_initial_jointPos5, "0");
  
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ArmImageGenerator::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmImageGenerator::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmImageGenerator::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ArmImageGenerator::onActivated(RTC::UniqueId ec_id)
{
	std::cout << "[ArmImageGenerator] Initializing Arm and Parameters" << std::endl;

	coil::TimeValue tv1(5.0);
	coil::sleep(tv1);

	std::cout << "[ArmImageGenerator] Waiting Arm Component is Activated....." << std::endl;
	while (true) {
      //int ok_count = 0;
 		const RTC::PortProfile& pp = m_manipCommonPort.getPortProfile();
 		if (pp.connector_profiles.length() > 0) {
 			RTC::PortProfile_var pp0 = pp.connector_profiles[0].ports[0]->get_port_profile();
 			RTC::ComponentProfile_var cp0 = pp0->owner->get_component_profile();
 			if (std::string(cp0->type_name) == armimagegenerator_spec[1]) {
 				RTC::PortProfile_var pp1 = pp.connector_profiles[0].ports[1]->get_port_profile();
 				RTC::ComponentProfile_var cp1 = pp1->owner->get_component_profile();
 				RTC::ExecutionContext_var ec1 = pp1->owner->get_context(0);
 				if (ec1->get_component_state(pp1->owner) == ACTIVE_STATE) {
 					break;
 				}
 			}
 			else {
 				RTC::ExecutionContext_var ec0 = pp0->owner->get_context(0);
 				if (ec0->get_component_state(pp0->owner) == ACTIVE_STATE) {
 					break;
 				}
 			}
 		}
 	}


	JARA_ARM::RETURN_ID_var ret = m_manipCommon->servoON();
	if (ret->id != JARA_ARM::OK) {
	  std::cout << "ERROR in ServoON" << std::endl;
	  std::cout << " ERRORCODE    :" << ret->id << std::endl;
	  std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
	}

    // the speed of movement
 	ret = m_manipMiddle->setSpeedJoint(20);
	if (ret->id != JARA_ARM::OK) {
	  std::cout << "ERROR in ServoON" << std::endl;
	  std::cout << " ERRORCODE    :" << ret->id << std::endl;
	  std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
	}

    // set the initial position
 	m_jointPos->length(6);
 	m_jointPos[0] = m_initial_jointPos0;
 	m_jointPos[1] = m_initial_jointPos1;
 	m_jointPos[2] = m_initial_jointPos2;
 	m_jointPos[3] = m_initial_jointPos3;
 	m_jointPos[4] = m_initial_jointPos4;
 	m_jointPos[5] = m_initial_jointPos5;

 	//m_manipMiddle->movePTPJointAbs(m_jointPos);


 	coil::TimeValue tv(3.0);
 	coil::sleep(tv);


 	m_j0counter = m_j1counter = 0;

 	m_sleepTime = coil::TimeValue(m_wait_interval);
 	std::cout << "[ArmImageGenerator] Wait " << m_sleepTime.sec() << "[sec], " << m_sleepTime.usec() << "[usec]" << std::endl;
	
 	std::cout << "[ArmImageGenerator] Ready." << std::endl;

 	time_t now = std::time(NULL);
 	struct tm* localNow = std::localtime(&now);
 	std::ostringstream ss;
 	ss << "log"
 		<< 1900 + localNow->tm_year
 		<< std::setw(2) << std::setfill('0') << localNow->tm_mon + 1
 		<< std::setw(2) << std::setfill('0') << localNow->tm_mday
 		<< std::setw(2) << std::setfill('0') << localNow->tm_hour
 		<< std::setw(2) << std::setfill('0') << localNow->tm_min
 		<< std::setw(2) << std::setfill('0') << localNow->tm_sec;

 	m_logDir = ss.str();
 #ifdef WIN32
 	_mkdir(m_logDir.c_str());
 #else
 	mkdir(m_logDir.c_str(), 0777);
 #endif

 	/*
 	std::ofstream configFile;
 	configFile.open(m_logDir + "/config.yaml", std::ofstream::out);
 	configFile << "j0min: " << m_j0min << std::endl;
 	configFile << "j1min: " << m_j1min << std::endl;
 	configFile << "j0max: " << m_j0max << std::endl;
 	configFile << "j1max: " << m_j1max << std::endl;
 	configFile << "j0step: " << m_j0step << std::endl;
 	configFile << "j1step: " << m_j1step << std::endl;
 	configFile << "wait_interval: " << m_wait_interval << std::endl;
 	configFile.close();
 	*/
     std::string filename = m_logDir + "/joints.csv";
 	m_JointLog.open(filename.c_str(), std::ios::out);//, std::ofstream::out);

 	m_JointLog << "x, y, theta, ImageFilename" << std::endl;
    
 	return RTC::RTC_OK;
 }



 RTC::ReturnCode_t ArmImageGenerator::onDeactivated(RTC::UniqueId ec_id)
 {
 	m_jointPos[0] = 0;
 	m_jointPos[1] = 0;
 	m_jointPos[2] = M_PI/2;
 	m_jointPos[4] = M_PI/2;
	
 	JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPJointAbs(m_jointPos);
	if (ret->id != JARA_ARM::OK) {
	  std::cout << "ERROR in ServoON" << std::endl;
	  std::cout << " ERRORCODE    :" << ret->id << std::endl;
	  std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
	}
  
 	m_JointLog.close();

 	coil::TimeValue tv(3.0);
 	coil::sleep(tv);

 	m_manipCommon->servoOFF();
 	return RTC::RTC_OK;
 }

 double Uniform( void ){
   return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
 }


 RTC::ReturnCode_t ArmImageGenerator::onExecute(RTC::UniqueId ec_id)
 {
   // range of x-axis [mm]
   double xlimit[2] = {120, 240};
   // range of y-axis [mm]
   double ylimit[2] = {-120, 120};
   // range of theta [rad]
   double thlimit[2] = {-M_PI+1.0e-10, M_PI-1.0e-10};
  

   double x = Uniform() * (xlimit[1] - xlimit[0]) + xlimit[0];
   double y = Uniform() * (ylimit[1] - ylimit[0]) + ylimit[0];
   double th = Uniform() * (thlimit[1] - thlimit[0]) + thlimit[0];
   
   // [mm] -> [m]
   x /= 1000.0;
   y /= 1000.0;

   // range of z-axis [mm]/1000.0 -> [m]
   double z = 40 / 1000.0;
   double z_min = 10 / 1000.0;

   double s2 = sin(th);
   double c2 = cos(th);


   std::cout << "--------------------------------------------------" << std::endl;

   JARA_ARM::CarPosWithElbow carPos;

   std::cout << "Reach (" << x << ", " << y << ", " << z << ")" << std::endl;
   carPos.carPos[0][0] = -c2;  carPos.carPos[0][1] = s2; carPos.carPos[0][2] =  0.0; carPos.carPos[0][3] = x;
   carPos.carPos[1][0] =  s2;  carPos.carPos[1][1] = c2; carPos.carPos[1][2] =  0.0; carPos.carPos[1][3] = y;
   carPos.carPos[2][0] =  0.0; carPos.carPos[2][1] = 0; carPos.carPos[2][2] = -1.0; carPos.carPos[2][3] = z;
   carPos.elbow = 1.0;
   carPos.structFlag = 1;
   JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(carPos);
   if (ret->id != JARA_ARM::OK) {
     
     return RTC::RTC_OK;
     
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
   
   coil::sleep(m_sleepTime);

   std::cout << "[ArmImageGenerator] Down" << std::endl;
   carPos.carPos[2][3] = z_min;
   ret = m_manipMiddle->movePTPCartesianAbs(carPos);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
   
   coil::sleep(m_sleepTime);

   std::cout << "[ArmImageGenerator] Release" << std::endl;
   ret = m_manipMiddle->moveGripper(50);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
   
   coil::sleep(m_sleepTime);

   std::cout << "[ArmImageGenerator] Up" << std::endl;
   carPos.carPos[2][3] = z;
   ret = m_manipMiddle->movePTPCartesianAbs(carPos);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
   
   coil::sleep(m_sleepTime);

   std::cout << "[ArmImageGenerator] Escape" << std::endl;
   //  m_jointPos->length(6);
   m_jointPos[0] = m_camera_jointPos0;
   m_jointPos[1] = m_camera_jointPos1;
   m_jointPos[2] = m_camera_jointPos2;
   m_jointPos[3] = m_camera_jointPos3;
   m_jointPos[4] = m_camera_jointPos4;
   m_jointPos[5] = m_camera_jointPos5;
   
   ret = m_manipMiddle->movePTPJointAbs(m_jointPos);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
   
   coil::sleep(m_sleepTime);

   JARA_ARM::CarPosWithElbow_var actual(new JARA_ARM::CarPosWithElbow());
   //JARA_ARM::RETURN_ID_var ret2 = m_manipCommon->getFeedbackPosJoint(actual);
   
   ret = m_manipMiddle->getFeedbackPosCartesian(actual);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }

   std::cout << "[ArmImageGenerator] Waiting for CameraImage...." << std::ends;

   time_t now = std::time(NULL);
   struct tm* localNow = std::localtime(&now);
   std::ostringstream ss;
   ss 
     << "image"
     << 1900 + localNow->tm_year
     << std::setw(2) << std::setfill('0') << localNow->tm_mon + 1
     << std::setw(2) << std::setfill('0') << localNow->tm_mday
     << std::setw(2) << std::setfill('0') << localNow->tm_hour
     << std::setw(2) << std::setfill('0') << localNow->tm_min
     << std::setw(2) << std::setfill('0') << localNow->tm_sec
     << ".png";

   coil::sleep(m_camera_wait_time);
   
   std::string filename = ss.str();


 #if 1
   /// Capture Image and Save
   bool imageArrived = false;
   //long counter = 0;

  //Inport data check
  while (m_cameraIn.isNew() && (!imageArrived)) {
    m_cameraIn.read();
    imageArrived = true;
  }
  std::cout << "[ArmImageGenerator] Image Arrived." << std::endl;
  
  long width = m_camera.data.image.width;
  long height = m_camera.data.image.height;
  long channels = (m_camera.data.image.format == Img::CF_GRAY) ? 1 :
    (m_camera.data.image.format == Img::CF_RGB || m_camera.data.image.format == Img::CF_PNG || m_camera.data.image.format == Img::CF_JPEG) ? 3 :
    (m_camera.data.image.raw_data.length() / width / height);
  
  if (channels == 3)
    m_buffer.create(height, width, CV_8UC3);
  else
    m_buffer.create(height, width, CV_8UC1);
  
  long data_length = m_camera.data.image.raw_data.length();

  //long image_size = width * height * channels;
  
  if (m_camera.data.image.format == Img::CF_RGB) {
    for (int i = 0; i<height; ++i)
      memcpy(&m_buffer.data[i*m_buffer.step], &m_camera.data.image.raw_data[i*width*channels], sizeof(unsigned char)*width*channels);
    if (channels == 3)
      cv::cvtColor(m_buffer, m_buffer, CV_RGB2BGR);
  }
  else if (m_camera.data.image.format == Img::CF_JPEG || m_camera.data.image.format == Img::CF_PNG) {
    std::vector<uchar> compressed_image = std::vector<uchar>(data_length);
    memcpy(&compressed_image[0], &m_camera.data.image.raw_data[0], sizeof(unsigned char) * data_length);
    
    //Decode received compressed image
    cv::Mat decoded_image;
    if (channels == 3) {
      decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_COLOR);
      cv::cvtColor(decoded_image, m_buffer, CV_RGB2BGR);
    }
    else {
      decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_GRAYSCALE);
      m_buffer = decoded_image;
    }
  }
  
  cv::imwrite(m_logDir + "/" + filename, m_buffer);
  
#endif  

  m_JointLog << x << ", " << y << ", " << th << ", " << filename << std::endl;


  std::cout << "[ArmImageGenerator] Ready" << std::endl;
  m_jointPos[0] = m_initial_jointPos0;
  m_jointPos[1] = m_initial_jointPos1;
  m_jointPos[2] = m_initial_jointPos2;
  m_jointPos[3] = m_initial_jointPos3;
  m_jointPos[4] = m_initial_jointPos4;
  m_jointPos[5] = m_initial_jointPos5;
  m_manipMiddle->movePTPJointAbs(m_jointPos);  
  coil::sleep(m_sleepTime);

  std::cout << "[ArmImageGenerator] Reach" << std::endl;
  carPos.carPos[2][3] = z;
  ret = m_manipMiddle->movePTPCartesianAbs(carPos);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
  
  coil::sleep(m_sleepTime);

  std::cout << "[ArmImageGenerator] Down" << std::endl;
  carPos.carPos[2][3] = z_min;
  ret = m_manipMiddle->movePTPCartesianAbs(carPos);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
  
  coil::sleep(m_sleepTime);

  std::cout << "[ArmImageGenerator] Hold" << std::endl;

  double ratio = m_gripper_close_ratio > 1.0 ? 1.0 : m_gripper_close_ratio < 0.0 ? 0 : m_gripper_close_ratio;
  ret = m_manipMiddle->moveGripper(100 * ratio);  //moveGripper(10);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
  
  coil::sleep(m_sleepTime);

  std::cout << "[ArmImageGenerator] Up" << std::endl;
  carPos.carPos[2][3] = z;
  ret = m_manipMiddle->movePTPCartesianAbs(carPos);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
  
  coil::sleep(m_sleepTime);

  std::cout << "[ArmImageGenerator] Ready" << std::endl;
  m_jointPos[0] = m_initial_jointPos0;
  m_jointPos[1] = m_initial_jointPos1;
  m_jointPos[2] = m_initial_jointPos2;
  m_jointPos[3] = m_initial_jointPos3;
  m_jointPos[4] = m_initial_jointPos4;
  m_jointPos[5] = m_initial_jointPos5;
  
  ret = m_manipMiddle->movePTPJointAbs(m_jointPos);
   if (ret->id != JARA_ARM::OK) {
     std::cout << "ERROR in ServoON" << std::endl;
     std::cout << " ERRORCODE    :" << ret->id << std::endl;
     std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
   }
  
  coil::sleep(m_sleepTime);
  std::cout << "------------------------------------------------------------" << std::endl;


  m_JointLog.flush();
  
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t ArmImageGenerator::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmImageGenerator::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmImageGenerator::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmImageGenerator::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmImageGenerator::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void ArmImageGeneratorInit(RTC::Manager* manager)
  {
    coil::Properties profile(armimagegenerator_spec);
    manager->registerFactory(profile,
                             RTC::Create<ArmImageGenerator>,
                             RTC::Delete<ArmImageGenerator>);
  }
  
};


