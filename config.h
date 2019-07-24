#ifndef CONFIG_H_
#define CONFIG_H_

/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/****                                  可配置参数                                              ****/
/*************************************************************************************************/

/* this file consists of several sections
 * 这个文件由几个部分组成
 * to create a working combination you must at least make your choices in section 1.
 * 要创建一个工作组合，您必须至少在1节中做出选择。
 * 1 - BASIC SETUP - you must select an option in every block.
 * 1 - 基本设置 - 您在基本设置中必须选择一个选项
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 以你连接了4个通道的标准的电调和电机为例。
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 2 - 飞行器类型特定的选项，你可能要检查你的飞行器类型选项的设置
 * 3 - RC SYSTEM SETUP
 * 3 - 无线遥控系统的设置
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 4 - 替代的CPU和主板 - 如果你有
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 5 - 替代设置 - 选择替代的RX（SBU，PPM，等），替代ESC范围，等在这里
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 6 - 可选功能 - 这里有一些很好的功能可以启用（飞行的模式，LCD，遥测，电池监控等）
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 * 7 - 调试和开发 - 如果你知道你正在做什么，已经警告过你了。
 *     - (ESCs calibration, Dynamic Motor/Prop Balancing, Diagnostics,Memory savings.....)
 * .. - (ESC的动态校准，电机/支撑平衡，诊断，节省内存.....)
 * 8 - DEPRECATED - these features will be removed in some future release
 * 8 - 不推荐使用 - 这些功能将在将来的版本中删除
 * 
 */

/* Notes:
 * 1. parameters marked with (*) in the comment are stored in eeprom and can be changed via serial monitor or LCD.
 * 1. 在注释中用(*)标记的参数被储存在eeprom中，并且可以通过串口监控器或LCD调节。
 * 2. parameters marked with (**) in the comment are stored in eeprom and can be changed via the GUI
 * 2. 在注释中用(**)标记的参数被储存在eeprom中，并且可以通过GUI调节
 */

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP   基本设置                                      *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/**************************    The type of multicopter    ****************************/
/**************************    多旋翼飞行器种类    ****************************/
//#define GIMBAL //自稳云台
//#define BI //两轴
//#define TRI //三轴
//#define QUADP //四轴十字模式
//#define QUADX //四轴X模式
//#define Y4 //四轴Y模式
//#define Y6 //六轴Y模式
//#define HEX6 //六轴
//#define HEX6X //六轴X模式
//#define HEX6H  // New Model // 新类型 六轴H模式
//#define OCTOX8  //八轴
//#define OCTOFLATP //八轴十字
//#define OCTOFLATX //八轴X
//#define FLYING_WING //飞翼
//#define VTAIL4 //四轴v尾
//#define AIRPLANE //固定翼
//#define SINGLECOPTER //单旋翼
//#define DUALCOPTER //双旋翼
//#define HELI_120_CCPM //120度CCPM直升机
//#define HELI_90_DEG //90度斜盘直升机

/****************************    Motor minthrottle    *******************************/
/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
/* 设定发送至电调（ESC，Electronic Speed Controller）的最小油门命令
   该最小值允许电机运行在怠速上 即维持电机怠速的最低油门值.  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A // 用于Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A // 用于Super Simple ESCs 10A
//#define MINTHROTTLE 1064 // special ESC (simonk) // 特殊的ESC (simonk蜘蛛电调)
//#define MINTHROTTLE 1050 // for brushed ESCs like ladybird // 用于brushed ESC比如ladybird
#define MINTHROTTLE 1150 // (*) (**)

/****************************    Motor maxthrottle    *******************************/
/* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    /****************************    电机最大油门     *******************************/
    /* ESC全功率工作的最大值，该值最大可增至2000 */
#define MAXTHROTTLE 1850

/****************************    Mincommand          *******************************/
/* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    /****************************    最小命令          *******************************/
    /* 该值用于未解锁时的ESC
      在某些情况下，用于一些特殊的电调该值必须降至900，否则电调会初始化失败
      OYUZIQI提示 用场效应管驱动空心杯时上电若微微转动可适当降低此数值
     */
#define MINCOMMAND 1000

/**********************************  I2C speed for old WMP config (useless config for other sensors)  *************/
/**********************************    I2C速度   ************************************/
#define I2C_SPEED 100000L //100kHz normal mode, this value must be used for a genuine WMP //100kHz普通模式，正品WPM必须使用该值
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones // 400kHz快速模式，仅用于一些山寨WPM （可用于GY系列模块）

/***************************    Internal i2c Pullups   ********************************/
/* enable internal I2C pull ups (in most cases it is better to use external pullups) */
/***************************    内部i2c上拉        ********************************/
/* 启用内部I2C上拉（在多数情况下，使用外部上拉更佳）（GY系列模块自带外部上拉） */
//#define INTERNAL_I2C_PULLUPS

/**********************************  constant loop time  ******************************/
/**********************************  循环周期时间  ******************************/
#define LOOP_TIME 2800

/**************************************************************************************/
/*****************          boards and sensor definitions            ******************/
/**************************************************************************************/
/*****************          主控板与传感器定义            ******************/

/***************************    Combined IMU Boards    ********************************/
/***************************    传感器组合板（传感器集成板）    ********************************/
/* if you use a specific sensor board:
         please submit any correction to this list.
           Note from Alex: I only own some boards, for other boards, I'm not sure, the info was gathered via rc forums, be cautious */
/*** 如果你在使用特定的传感器板：
    请提交改动到这个列表。
    来自Alex的提示：我只有其中一些板子，对于其他板子，我不能确保好用，信息由遥控论坛生成，请小心使用 
  ****/
//#define FFIMUv1         // first 9DOF+baro board from Jussi, with HMC5843                   <- confirmed by Alex
//#define FFIMUv2         // second version of 9DOF+baro board from Jussi, with HMC5883       <- confirmed by Alex
//#define FREEIMUv1       // v0.1 & v0.2 & v0.3 version of 9DOF board from Fabio
//#define FREEIMUv03      // FreeIMU v0.3 and v0.3.1
//#define FREEIMUv035     // FreeIMU v0.3.5 no baro
//#define FREEIMUv035_MS  // FreeIMU v0.3.5_MS                                                <- confirmed by Alex
//#define FREEIMUv035_BMP // FreeIMU v0.3.5_BMP
//#define FREEIMUv04      // FreeIMU v0.4 with MPU6050, HMC5883L, MS561101BA                  <- confirmed by Alex
//#define FREEIMUv043     // same as FREEIMUv04 with final MPU6050 (with the right ACC scale)
//#define NANOWII         // the smallest multiwii FC based on MPU6050 + pro micro based proc <- confirmed by Alex
//#define PIPO            // 9DOF board from erazz
//#define QUADRINO        // full FC board 9DOF+baro board from witespy  with BMP085 baro     <- confirmed by Alex
//#define QUADRINO_ZOOM   // full FC board 9DOF+baro board from witespy  second edition
//#define QUADRINO_ZOOM_MS// full FC board 9DOF+baro board from witespy  second edition       <- confirmed by Alex
//#define QUADRINO_NANO   // full FC board 9DOF+baro board+GPS from Lynxmotion / RobotShop    <- confirmed by Alex
//#define ALLINONE        // full FC board or standalone 9DOF+baro board from CSG_EU
//#define AEROQUADSHIELDv2
//#define ATAVRSBIN1      // Atmel 9DOF (Contribution by EOSBandi). requires 3.3V power.
//#define SIRIUS          // Sirius Navigator IMU                                             <- confirmed by Alex
//#define SIRIUSGPS       // Sirius Navigator IMU  using external MAG on GPS board            <- confirmed by Alex
//#define SIRIUS600       // Sirius Navigator IMU  using the WMP for the gyro
//#define SIRIUS_AIR      // Sirius Navigator IMU 6050 32U4 from MultiWiiCopter.com           <- confirmed by Alex
//#define SIRIUS_AIR_GPS  // Sirius Navigator IMU 6050 32U4 from MultiWiiCopter.com with GPS/MAG remote located
//#define SIRIUS_MEGAv5_OSD //  Paris_Sirius™ ITG3050,BMA280,MS5611,HMC5883,uBlox  http://www.Multiwiicopter.com <- confirmed by Alex
//#define MINIWII         // Jussi's MiniWii Flight Controller                                <- confirmed by Alex
//#define MICROWII        // MicroWii 10DOF with ATmega32u4, MPU6050, HMC5883L, MS561101BA from http://flyduino.net/
//#define CITRUSv2_1      // CITRUS from qcrc.ca
//#define CHERRY6DOFv1_0
//#define DROTEK_10DOF    // Drotek 10DOF with ITG3200, BMA180, HMC5883, BMP085, w or w/o LLC
//#define DROTEK_10DOF_MS // Drotek 10DOF with ITG3200, BMA180, HMC5883, MS5611, LLC
//#define DROTEK_6DOFv2   // Drotek 6DOF v2
//#define DROTEK_6DOF_MPU // Drotek 6DOF with MPU6050
//#define DROTEK_10DOF_MPU//
//#define MONGOOSE1_0     // mongoose 1.0    http://store.ckdevices.com/
//#define CRIUS_LITE      // Crius MultiWii Lite
//#define CRIUS_SE        // Crius MultiWii SE
//#define CRIUS_SE_v2_0   // Crius MultiWii SE 2.0 with MPU6050, HMC5883 and BMP085
//#define OPENLRSv2MULTI  // OpenLRS v2 Multi Rc Receiver board including ITG3205 and ADXL345
//#define BOARD_PROTO_1   // with MPU6050 + HMC5883L + MS baro
//#define BOARD_PROTO_2   // with MPU6050 + slave  MAG3110 + MS baro
//#define GY_80           // Chinese 10 DOF with  L3G4200D ADXL345 HMC5883L BMP085, LLC
//#define GY_85           // Chinese 9 DOF with  ITG3205 ADXL345 HMC5883L LLC
//#define GY_86           // Chinese 10 DOF with  MPU6050 HMC5883L MS5611, LLC
//#define GY_88 // Chinese 10 DOF with MPU6050 HMC5883L BMP085, LLC
//#define GY_521          // Chinese 6  DOF with  MPU6050, LLC
//#define INNOVWORKS_10DOF // with ITG3200, BMA180, HMC5883, BMP085 available here http://www.diymulticopter.com
//#define INNOVWORKS_6DOF // with ITG3200, BMA180 available here http://www.diymulticopter.com
//#define MultiWiiMega    // MEGA + MPU6050+HMC5883L+MS5611 available here http://www.diymulticopter.com
//#define PROTO_DIY       // 10DOF mega board
//#define IOI_MINI_MULTIWII// www.bambucopter.com
//#define Bobs_6DOF_V1     // BobsQuads 6DOF V1 with ITG3200 & BMA180
//#define Bobs_9DOF_V1     // BobsQuads 9DOF V1 with ITG3200, BMA180 & HMC5883L
//#define Bobs_10DOF_BMP_V1 // BobsQuads 10DOF V1 with ITG3200, BMA180, HMC5883L & BMP180 - BMP180 is software compatible with BMP085
//#define FLYDUINO_MPU       // MPU6050 Break Out onboard 3.3V reg
//#define CRIUS_AIO_PRO
//#define DESQUARED6DOFV2GO  // DEsquared V2 with ITG3200 only
//#define DESQUARED6DOFV4    // DEsquared V4 with MPU6050
//#define LADYBIRD
//#define MEGAWAP_V2_STD     // available here: http://www.multircshop.com                    <- confirmed by Alex
//#define MEGAWAP_V2_ADV
//#define HK_MultiWii_SE_V2  // Hobbyking board with MPU6050 + HMC5883L + BMP085
//#define HK_MultiWii_328P   // Also labeled "Hobbybro" on the back.  ITG3205 + BMA180 + BMP085 + NMC5583L + DSM2 Connector (Spektrum Satellite)
//#define RCNet_FC           // RCNet FC with MPU6050 and MS561101BA  http://www.rcnet.com
//#define RCNet_FC_GPS       // RCNet FC with MPU6050 + MS561101BA + HMC5883L + UBLOX GPS http://www.rcnet.com
//#define FLYDU_ULTRA        // MEGA+10DOF+MT3339 FC
//#define DIYFLYING_MAGE_V1  // diyflying 10DOF mega board with MPU6050 + HMC5883L + BMP085 http://www.indoor-flying.hk
//#define MultiWii_32U4_SE         // Hextronik MultiWii_32U4_SE
//#define MultiWii_32U4_SE_no_baro // Hextronik MultiWii_32U4_SE without the MS561101BA to free flash-memory for other functions
//#define Flyduino9DOF       // Flyduino 9DOF IMU MPU6050+HMC5883l
//#define Nano_Plane         // Multiwii Plane version with tail-front LSM330 sensor http://www.radiosait.ru/en/page_5324.html

/***************************    independent sensors    ********************************/
/***************************    独立的传感器    ********************************/
/* leave it commented if you already checked a specific board above */
/*** 如果你已在上方选择了相应的组合板子，请跳过，保持以下注释状态即可 
 *这里是用来设置你单独连接在I2C上的传感器模块。当然每样都单独买价格会高一些，如果你是豪，当我没说。
****/
/* I2C gyroscope */ /* I2C陀螺仪 */
//#define WMP
//#define ITG3050
//#define ITG3200
//#define MPU3050
//#define L3G4200D
//#define MPU6050       //combo + ACC // 带了加速度
//#define LSM330        //combo + ACC

/* I2C accelerometer */ /* I2C加速度计 */
//#define MMA7455
//#define ADXL345
//#define BMA020
//#define BMA180
//#define BMA280
//#define LIS3LV02
//#define LSM303DLx_ACC
//#define MMA8451Q

/* I2C barometer */ /* I2C气压计 */
//#define BMP085
//#define MS561101BA

/* I2C magnetometer */ /* I2C磁力计 */
//#define HMC5843
//#define HMC5883
//#define AK8975
//#define MAG3110

/* Sonar */ // for visualization purpose currently - no control code behind
/* 声呐 */  // 目前用作显示用途 - 无控制代码支持
/* OYUZIQI提示：貌似bbs.5imx.com上有大神完成了代码，且国外论坛上有光流
 205 http://bbs.5imx.com/bbs/forum.php?mod=viewthread&tid=726172&page=1
 206 http://www.multiwii.com/forum/viewtopic.php?f=7&t=1413
 207 */
//#define SRF02 // use the Devantech SRF i2c sensors
//#define SRF08
//#define SRF10
//#define SRF23

/* ADC accelerometer */ // for 5DOF from sparkfun, uses analog PIN A1/A2/A3
/* ADC加速度计 */ // 用于来自sparkfun的5DOF，使用模拟针脚A1/A2/A3
//#define ADCACC

/* enforce your individual sensor orientation - even overrides board specific defaults */
/* 强制你独有的的传感器方向 - 甚至覆盖集成dof板子特定的默认值 */
//#define FORCE_ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  =  Y; imu.accADC[PITCH]  = -X; imu.accADC[YAW]  = Z;}
//#define FORCE_GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = -Y; imu.gyroADC[PITCH] =  X; imu.gyroADC[YAW] = Z;}
//#define FORCE_MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = Z;}

/* Board orientation shift */
/* 板子方向转移 */
/* If you have frame designed only for + mode and you cannot rotate FC phisycally for flying in X mode (or vice versa)
       * you can use one of of this options for virtual sensors rotation by 45 deegres, then set type of multicopter according to flight mode.
       * Check motors order and directions of motors rotation for matching with new front point!  Uncomment only one option! */
/* 如果你的机架设计仅用于+模式，并且你不能物理上将飞控旋转至用于X模式飞行（反之亦然）
  你可以使用其中一个选项虚拟旋转传感器45度，然后通过飞行模式设定多旋翼飞行器的类型。
  检查电机顺序与旋转方向是否与新的“前方”匹配！仅使用其中一项注释！ */
//#define SENSORS_TILT_45DEG_RIGHT        // rotate the FRONT 45 degres clockwise // 将“前方”顺时针旋转45度
//#define SENSORS_TILT_45DEG_LEFT         // rotate the FRONT 45 degres counterclockwise // 将“前方”逆时针旋转45度


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                飞行器类型特定的选项                              ***************/
/*************************************************************************************************/
  /********************************  PID Controller  PID控制算法*********************************/
    /* choose one of the alternate PID control algorithms
     * 1 = evolved oldschool algorithm (similar to v2.2)
     * 2 = new experimental algorithm from Alex Khoroshko - unsupported - http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=10#p37387
     * */
    /* 单独选择一个PID控制算法
       1.演进 oldschool 算法（类似于V2.2）
       2.新的实验算法 来自 Alex Khoroshko - 无技术支持 - http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=10#p37387
    */
    #define PID_CONTROLLER 1

    /* NEW: not used anymore for servo coptertypes  <== NEEDS FIXING - MOVE TO WIKI */
    /* NEW: 不再使用伺服飞行器种类 (需要修复)）   <== NEEDS FIXING - MOVE TO WIKI */
    #define YAW_DIRECTION 1
    //#define YAW_DIRECTION -1 // if you want to reverse the yaw correction direction // 如果你想反向修正偏航方向

    #define ONLYARMWHENFLAT //prevent the copter from arming when the copter is tilted // 阻止飞行器倾斜时解锁

   /********************************    ARM/DISARM  锁定/解锁  *********************************/
   /* optionally disable stick combinations to arm/disarm the motors.
     -* In most cases one of the two options to arm/disarm via TX stick is sufficient */
     /* 可以禁止使用摇杆组合进行锁定/解锁电机。
    -* 在多数情况下，选择其中一种通过发射机摇杆锁定/解锁电机的选项即可 */
    #define ALLOW_ARM_DISARM_VIA_TX_YAW
    //#define ALLOW_ARM_DISARM_VIA_TX_ROLL

    /********************************    SERVOS    舵机  *********************************/
    /* info on which servos connect where and how to setup can be found here // 舵机连接在哪里以及如何设置可以在这里找到 
     * http://www.multiwii.com/wiki/index.php?title=Config.h#Servos_configuration
     */

    /* Do not move servos if copter is unarmed //如果没有装，不要动伺服系统部分。
     * It is a quick hack to overcome feedback tail wigglight when copter has a flexibile
     * landing gear
    */
    //#define DISABLE_SERVOS_WHEN_UNARMED


    /* if you want to preset min/middle/max values for servos right after flashing, because of limited physical
     -* room for servo travel, then you must enable and set all three following options */
    /**** 如果你想预定最小/中间/最大值为伺服正确后flashing，
     因为物理因素限制伺服行程的设置，您必须启用并设置所有三以下选项 *** /
     //#define SERVO_MIN  {1020, 1020, 1020, 1020, 1020, 1020, 1020, 1020}  //舵机最小值
     //#define  SERVO_MAX {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000}  //舵机大值
     //#define  SERVO_MID {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500} // (*)  // (*)舵机中立点
     //#define FORCE_SERVO_RATES      {30,30,100,100,100,100,100,100} // 0 = normal, 1= reverse  // 0=正向 1=反向

  /***********************          Cam Stabilisation             ***********************/
    /* The following lines apply only for a pitch/roll tilt stabilization system. Uncomment the first or second line to activate it */
    /***********************          相机稳定             ***********************/
    /* 以下几行仅用于pitch/roll倾斜稳定系统。去除注释第一或第二行来激活它 ***/
    //#define SERVO_MIX_TILT
    //#define SERVO_TILT
    /* camera trigger function : activated via Rc Options in the GUI, servo output=A2 on promini */
    // trigger interval can be changed via (*GUI*) or via AUX channel
    /*** 相机触发设置 : 激发路径显示在GUI, 使用A2作为舵机输出在promini ***/
    // 触发路径可以设置 (*GUI*) or 或者通过AUX铺助通道
    //#define CAMTRIG
    #define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms  // 高电平时间（毫秒）

  /***********************          Airplane                       ***********************/
    //#define USE_THROTTLESERVO // For use of standard 50Hz servo on throttle.  // 用于在油门上使用标准50Hz舵机。

    //#define FLAPPERONS    AUX4          // Mix Flaps with Aileroins. // 混合襟翼与副翼。
    #define FLAPPERON_EP   { 1500, 1700 } // Endpooints for flaps on a 2 way switch else set {1020,2000} and program in radio. // 用于襟翼双向切换的端点，另外可设为{1020,2000}并在遥控上编程。
    #define FLAPPERON_INVERT { -1, 1 }    // Change direction om flapperons { Wing1, Wing2 } // 改变襟副翼的方向{ Wing1, Wing2 }
    
    //#define FLAPS                       // Traditional Flaps on SERVO3. // 传统移动 SERVO3.
    //#define FLAPSPEED     3             // Make flaps move slowm Higher value is Higher Speed. //使襟翼移动变慢，值越高速度越快。

  /***********************      Common for Heli & Airplane    直升机与飞机通用    ***********************/

    /* Governor: attempts to maintain rpm through pitch and voltage changes
     * predictive approach: observe input signals and voltage and guess appropriate corrections.
     * (the throttle curve must leave room for the governor, so 0-50-75-80-80 is ok, 0-50-95-100-100 is _not_ ok.
     * Can be toggled via aux switch.
     */
    /* 调节器：试图通过螺距和电压的改变维持转速
     -* 预测方法：观察输入信号与电压并猜测适当的修正。
     -* （油门曲线必须为调节器留有空间，所以0-50-75-80-80是可以的，不可以为0-50-95-100-100。
     -* 可以通过aux开关切换
     */
    //#define GOVERNOR_P 7     // (*) proportional factor. Higher value -> higher throttle increase. Must be >=1; 0 = turn off // (*) 比例因子。更大的值 -> 更大的油门增量。必须>=1；0 = 关闭
    //#define GOVERNOR_D 4     // (*) decay timing. Higher value -> takes longer to return throttle to normal. Must be >=1; // (*) 衰减时间。更大的值 -> 油门回到正常需要更长时间。 必须>=1；

    /* tail precomp from collective */
    #define YAW_COLL_PRECOMP 10           // (*) proportional factor in 0.1. Higher value -> higher precomp effect. value of 10 equals no/neutral effect
    #define YAW_COLL_PRECOMP_DEADBAND 120 // (*) deadband for collective pitch input signal around 0-pitch input value

    //#define VOLTAGEDROP_COMPENSATION // voltage impact correction // 电压影响校正

  /***********************          Heli     直升机             ***********************/
    /* Channel to control CollectivePitch */  /*  控制总距的通道 */
    #define COLLECTIVE_PITCH      THROTTLE

    /* Limit the range of Collective Pitch. 100% is Full Range each way and position for Zero Pitch */
    /* 限制总距的范围。100%为每个方向的最大范围，还有零螺距的位置 */
    #define COLLECTIVE_RANGE { 80, 0, 80 }// {Min%, ZeroPitch offset from 1500, Max%}. // {最小%,从1500开始的零螺距偏移,最大%}。
    #define YAWMOTOR                 0       // If a motor is used as YAW Set to 1 else set to 0. // 如果一个电机用作YAW则设为1，否则设为0。

    /* Servo mixing for heli 120
                         {Coll,Nick,Roll} */
    /* 用于120直升机的舵机混控，使用分数1/10（例.5 = 5/10 = 1/2）***/
    #define SERVO_NICK   { +10, -10,  0 }
    #define SERVO_LEFT   { +10, +5, +10 } 
    #define SERVO_RIGHT  { +10, +5, -10 } 

    /* Limit Maximum controll for Roll & Nick  in 0-100% */
    /* 限制用于Roll & Nick最大控制，范围0-100% */
    #define CONTROL_RANGE   { 100, 100 }      //  { ROLL,PITCH }

    /* use servo code to drive the throttle output. You want this for analog servo driving the throttle on IC engines.
       if inactive, throttle output will be treated as a motor output, so it can drive an ESC */
    /* 使用舵机代码驱动油门输出。用模拟舵机驱动IC引擎上的油门时，你会需要此项。
      如果不启用，油门输出会被看做电机输出，所以它可以驱动电调 */
    //#define HELI_USE_SERVO_FOR_THROTTLE

  /***********************      your individual mixing     ***********************/
    /* if you want to override an existing entry in the mixing table, you may want to avoid editing the
     * mixTable() function for every version again and again. 
     * howto: http://www.multiwii.com/wiki/index.php?title=Config.h#Individual_Mixing
     */
    /***********************      你的独立混控              ***********************/
    /* 如果你想要覆盖一个选存的混合表中的条目，你可能想要避免
    在每个版本一遍又一遍的编辑mixTable()函数 ****/
    //#define MY_PRIVATE_MIXING "filename.h"

  /***********************      your individual defaults     ***********************/
    /* if you want to replace the hardcoded default values with your own (e.g. from a previous save to an .mwi file),
     * you may want to avoid editing the LoadDefaults() function for every version again and again.
     * http://www.multiwii.com/wiki/index.php?title=Config.h#Individual_defaults
     */
    /***********************      你的默认参数    ***********************/
    /* 如果你想要覆盖一个选存的混合表中的条目，你可能想要避免
      在每个版本一遍又一遍的编辑 LoadDefaults() 函数
      *****/
    //#define MY_PRIVATE_DEFAULTS "filename.h"  // 更改filename.h为你自己的默认文件(jpno1注)


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/****************  SECTION  3 - 无线遥控系统设置                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /* note: no need to uncomment something in this section if you use a standard receiver */
  /* 提示：如果你使用的是标准接收机，不必取消本节的一些注释 */

/****************************    EXTENDED AUX STATES    ***********************************/
/* If you uncomment this line, you can use six states for each of the aux channels (AUX1-AUX4)
to control your copter.
Channel values
1000-1230
1231-1360
1361-1490
1491-1620
1621-1749
1750-
/****************************     扩展辅助状态     ***********************************/
/* 如果你启用这个设置，在AUX通道（aux1-aux4）通道，你可以使用六个档位。
 注意：能使用6个档位的只有wingui 2.3或者更新的版本地面站。multiwiiconf还不支持
 档位值
At this moment you can use this function only with WinGUI 2.3 release. MultiWiiConf does not support it yet
*/

//#define EXTENDED_AUX_STATES


  /**************************************************************************************/
  /********                       special receiver types             ********************/
  /********                       特殊接收机类型                     ********************/
  /**************************************************************************************/
  /****************************    PPM Sum接收机      ***********************************/

    /****************************    PPM Sum Reciver    ***********************************/
      /* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
         Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
      /* 下列几行仅用于特定的仅有一个PPM sum信号的接收机，接在数字针脚2上
       根据你的遥控品牌选择相应的行。当你的PPM顺序不同时，你可以随意修改顺序 ****/
      //#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
      //#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba
      //#define SERIAL_SUM_PPM         ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Multiplex
      //#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For some Hitec/Sanwa/Others //用于一些韩国的/日本三和/其它
      //#define SERIAL_SUM_PPM         THROTTLE,YAW,ROLL,PITCH,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //Modelcraft

      // Uncommenting following line allow to connect PPM_SUM receiver to standard THROTTLE PIN on MEGA boards (eg. A8 in CRIUS AIO)
      // 解除下面这行注释以允许连接PPM_SUM接收机至MEGA板上的标准油门针脚（例.CRIUS AIO上的A8）
      //#define PPM_ON_THROTTLE

    /**********************    Spektrum Satellite Reciver    *******************************/
    /**********************    Spektrum卫星接收机    *******************************/
      /* The following lines apply only for Spektrum Satellite Receiver
         Spektrum Satellites are 3V devices.  DO NOT connect to 5V!
         For MEGA boards, attach sat grey wire to RX1, pin 19. Sat black wire to ground. Sat orange wire to Mega board's 3.3V (or any other 3V to 3.3V source).
         For PROMINI, attach sat grey to RX0.  Attach sat black to ground. */
         /* 以下几行仅用于Spektrum卫星接收机
           Spektrum卫星系列是3V设备。不要连接至5V！
           对于MEGA板，将灰线连接到RX1，19针脚上。黑线接地。橙线连接到Mega板的3.3V上（或其他3V至3.3V的电源）。
           对于PROMINI，将灰线连接到RX0。黑线接地。 */
      //#define SPEKTRUM 1024
      //#define SPEKTRUM 2048
      //#define RX_SERIAL_PORT 1    // Forced to 0 on Pro Mini and single serial boards; Set to your choice of 0, 1, or 2 on any Mega based board (defaults to 1 on Mega).
                                    // Pro Mini与其他单串口的板子上只能设为0；在所有基于Mega的板子上设为你选择的0，1，2（在Mega上默认为1）。
      //**************************
      // Defines that allow a "Bind" of a Spektrum or Compatible Remote Receiver (aka Satellite) via Configuration GUI.
      //   Bind mode will be same as declared above, if your TX is capable.
      //   Ground, Power, and Signal must come from three adjacent pins. 
      //   By default, these are Ground=4, Power=5, Signal=6.  These pins are in a row on most MultiWii shield boards. Pins can be overriden below.  
      //   Normally use 3.3V regulator is needed on the power pin!!  If your satellite hangs during bind (blinks, but won't complete bind with a solid light), go direct 5V on all pins. 
// 定义此项允许Spektrum或兼容机远程接收机（也就是卫星）通过配置GUI对频。
      //   对频模式与上述的相同，只要你的发射机支持。
      //   接地，电源，信号必须来自三个邻近的针脚。
      //   默认下，它们为接地=4，电源=5，信号=6。这些针脚在多数MultiWii扩展板上都为一排。可在下方覆盖针脚。
      //   通常需要在电源针脚上使用3.3V稳压器！！如果你的卫星在对频时停摆（闪烁，但不会常亮停止闪烁），将所有的针脚连接至5V
      //**************************
      //   For Pro Mini, the connector for the Satellite that resides on the FTDI can be unplugged and moved to these three adjacent pins. 
      //   对于Pro Mini，用于卫星的属于FTDI的连接器可以拔掉，并移至那三个相邻针脚。
      //#define SPEK_BIND             //Un-Comment for Spektrum Satellie Bind Support.  Code is ~420 bytes smaller without it. //解除注释以开启Spektrum卫星对频支持。没有它代码可节省约420字节。
      //#define SPEK_BIND_GROUND 4
      //#define SPEK_BIND_POWER  5
      //#define SPEK_BIND_DATA   6

    /*******************************    SBUS RECIVER    ************************************/
      /* The following line apply only for Futaba S-Bus Receiver on MEGA boards or PROMICRO boards.
         You have to invert the S-Bus-Serial Signal e.g. with a Hex-Inverter like IC SN74 LS 04 */
    /* 下面这行仅用于Futaba S-Bus接收机在MEGA板上的RX1的情况（串口1）。
        你必须反转S-Bus-串口信号，例如使用十六进制反相器像是IC SN74 LS 04 */
      //#define SBUS     PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17  // dsm2 orangerx
      //#define SBUS     ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17  // T14SG
      //#define RX_SERIAL_PORT 1
      #define SBUS_MID_OFFSET 988 //SBUS Mid-Point at 1500

    /******************************* HOTT RECIVER    HOTT 接收机************************************/
    /* Graupner Hott HD */
    //#define SUMD PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4
    //#define RX_SERIAL_PORT 1

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  4 - ALTERNATE CPUs & BOARDS      替代的CPU和主板                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************************************************************************/
  /********                      Promini Specifig Settings           ********************/
  /********                      Promini板专用设置           ********************/
  /**************************************************************************************/

    /**************************    Hexa Motor 5 & 6 Pins    *******************************/
    /**************************       六轴电机 5 & 6 针脚    *******************************/
      /* PIN A0 and A1 instead of PIN D5 & D6 for 6 motors config and promini config
         This mod allow the use of a standard receiver on a pro mini
         (no need to use a PPM sum receiver) */
/****   用A0与A1针脚代替D5与D6针脚，用于6个电机配置与promini配置
     该模式允许在promini上标准接收机的使用
    （不必使用PPM sum接收机） ****/
      //#define A0_A1_PIN_HEX

    /*********************************    Aux 2 Pin     ***********************************/
      /* possibility to use PIN8 or PIN12 as the AUX2 RC input (only one, not both)
         it deactivates in this case the POWER PIN (pin 12) or the BUZZER PIN (pin 8) */
       /* 让你可以使用针脚8或针脚12作为遥控的AUX2输入（只可启用一个，不可全部启用）
          如果启用它会使功率针脚（针脚12）或蜂鸣针脚（针脚8）失效 */
      //#define RCAUXPIN8
      //#define RCAUXPIN12


  /**************************************************************************************/
  /*****************             Teensy 2.0 Support                    ******************/
  /**************************************************************************************/
    /* uncomment this if you use a teensy 2.0 with teensyduino
       it needs to run at 16MHz */
    /* 解除此项如果你使用的是使用teensyduino的teensy 2.0 它需要运行在16MHz */
    //#define TEENSY20


  /**************************************************************************************/
  /********   Settings for ProMicro, Leonardo and other Atmega32u4 Boards     ***********/
  /********   用于ProMicro，Leonardo和其他Atmega32u4板子的设置                ***********/
  /**************************************************************************************/

    /*********************************    pin Layout     **********************************/
      /* activate this for a better pinlayout if all pins can be used => not possible on ProMicro */
      /* 如果所有针脚都能使用，激活此项可获得更好的针脚布局 => 在ProMicro上不可用 */
      //#define A32U4ALLPINS

    /**********************************    PWM Setup     **********************************/
      /* activate all 6 hardware PWM outputs Motor 5 = D11 and 6 = D13. 
         note: not possible on the sparkfun promicro (pin 11 & 13 are not broken out there)
         if activated:
         Motor 1-6 = 10-bit hardware PWM
         Motor 7-8 = 8-bit Software PWM
         Servos    = 8-bit Software PWM
         if deactivated:
         Motor 1-4 = 10-bit hardware PWM
         Motor 5-8 = 10-bit Software PWM
         Servos    = 10-bit Software PWM */
      /*** 激活全部6个硬件PWM输出，电机5 = D11，电机6 = D13。
        提示：不可用于sparkfun promicro（针脚11 & 13未被引出）
        如果激活：
        电机1-6 = 10位硬件PWM
        电机7-8 = 8位软件PWM
        舵机    = 8位软件PWM
        如果未激活：
        电机1-4 = 10位硬件PWM
        电机5-8 = 10位软件PWM
        舵机    = 10位软件PWM ****/
      //#define HWPWM6

    /**********************************    Aux 2 Pin     **********************************/
      /* AUX2 pin on pin RXO */
      /* AUX2针脚在RXO针脚上 */
      //#define RCAUX2PINRXO

      /* aux2 pin on pin D17 (RXLED) */
      /* aux2针脚在D17针脚上（RXLED） */
      //#define RCAUX2PIND17

    /**********************************    Buzzer Pin   蜂鸣针脚 *******************************/
      /* this moves the Buzzer pin from TXO to D8 for use with ppm sum or spectrum sat. RX (not needed if A32U4ALLPINS is active) */
      /* 此项将蜂鸣针脚从TX0移动至D8以使用ppm sum或spectrum sat.接收机（如果启用了A32U4ALLPINS则不需此项） */
      //#define D8BUZZER

    /***********************      Promicro version related     ****************************/
      /* Inverted status LED for Promicro ver 10 */
      /* 反转状态LED用于Promicro版本10 */
      //#define PROMICRO10


  /**************************************************************************************/
  /********                      override default pin assignments    ********************/
  /********                      默认针脚分配调整    ********************/
  /**************************************************************************************/

  /* only enable any of this if you must change the default pin assignment, e.g. your board does not have a specific pin */
  /* you may need to change PINx and PORTx plus #shift according to the desired pin! */
  /* 仅在你必须改变默认针脚分配时才启用其中一项，例：你的板子没有特定针脚 */
  /* 你可能需要依据期望的针脚给PINx与PORTx加上#移位！ */
  //#define OVERRIDE_V_BATPIN                   A0 // instead of A3    // Analog PIN 3  // 代替A3    //模拟针脚3

  //#define OVERRIDE_PSENSORPIN                 A1 // instead of A2    // Analog PIN 2  // 代替A2    //模拟针脚2

  //#define OVERRIDE_LEDPIN_PINMODE             pinMode (A1, OUTPUT); // use A1 instead of d13  // 使用A1代替d13
  //#define OVERRIDE_LEDPIN_TOGGLE              PINC |= 1<<1; // PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13) //切换LED针脚状态（数字针脚13）
  //#define OVERRIDE_LEDPIN_OFF                 PORTC &= ~(1<<1); // PORTB &= ~(1<<5); 
  //#define OVERRIDE_LEDPIN_ON                  PORTC |= 1<<1;    // was PORTB |= (1<<5);

  //#define OVERRIDE_BUZZERPIN_PINMODE          pinMode (A2, OUTPUT); // use A2 instead of d8 // 使用A2代替d8
  //#define OVERRIDE_BUZZERPIN_ON               PORTC |= 1<<2 //PORTB |= 1;
  //#define OVERRIDE_BUZZERPIN_OFF              PORTC &= ~(1<<2); //PORTB &= ~1;

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/****************  SECTION  5 - 替代设置                                                   *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                Serial com speed    *********************************/
  /******                串行速率    *********************************/
    /* This is the speed of the serial interfaces */
    /* 此为每个串口的速率 */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200
    #define SERIAL2_COM_SPEED 115200
    #define SERIAL3_COM_SPEED 115200

    /* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
       it is relevent only for a conf with at least a WMP */
       /* 当I2C总线有错误时，我们可在很短的时间内中立化相关值。用微秒表示
        它仅与至少有一个WMP的配置相关 */
    #define NEUTRALIZE_DELAY 100000

  /**************************************************************************************/
  /********                             Gyro filters    陀螺仪滤波器  ********************/
  /**************************************************************************************/

    /*********************    Lowpass filter for some gyros    ****************************/
      /* ITG3200 & ITG3205 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
         to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
         It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
         balancing options ran out. Uncomment only one option!
         IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.
         available for ITG3050, ITG3200, MPU3050, MPU6050*/
/*********************    特定的几款陀螺仪的低通滤波器    ****************************/
 /* ITG3200 & ITG3205 的低通滤波设置. 如果你不能减小飞行器震动，你可以尝试
    逐步降低低通滤波器的频率，一旦抖动消失就可以保持相应滤波设置
    它对回馈引起的摆动不起作用，所以只在飞行器随机抽动并且所有抑制和平衡设置失效的时候才修改它。只取消注释其中一项！
    重要！改变低通滤波器设置将会改变PID的行为，所以在改变LPF后重新调整你的PID。
    支持低通滤波的陀螺仪模块：ITG3050, ITG3200, MPU3050, MPU6050*/
      //#define GYRO_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference  // 此为默认设置，不需要取消注释，只作为参考
      //#define GYRO_LPF_188HZ
      //#define GYRO_LPF_98HZ
      //#define GYRO_LPF_42HZ
      //#define GYRO_LPF_20HZ
      //#define GYRO_LPF_10HZ
      //#define GYRO_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props -- setting not available on ITG3200 // 只在极端情况下使用此项，更应该换电机和/或螺旋桨 -- 此设置不能在ITG3200陀螺仪上工作

    /******                Gyro smoothing    陀螺仪平滑化**********************************/
      /* GYRO_SMOOTHING. In case you cannot reduce vibrations _and_ _after_ you have tried the low pass filter options, you
         may try this gyro smoothing via averaging. Not suitable for multicopters!
         Good results for helicopter, airplanes and flying wings (foamies) with lots of vibrations.*/
    /* GYRO_SMOOTHING.在你不能消除振动的情况下，_并且_是在尝试了低通滤波器选项_之后_，你
       可尝试此通过平均化的陀螺仪平滑化。不适用于多旋翼飞行器！
       在有很多振动的直升机，飞机和飞翼（泡沫的）上可获得良好结果。*/
      //#define GYRO_SMOOTHING {20, 20, 3}    // (*) separate averaging ranges for roll, pitch, yaw

    /************************    Moving Average Gyros    **********************************/
    /************************    移动平均陀螺仪    **********************************/
      //#define MMGYRO 10                      // (*) Active Moving Average Function for Gyros // (*) 激活用于陀螺仪的移动平均函数
      //#define MMGYROVECTORLENGTH 15          // Length of Moving Average Vector (maximum value for tunable MMGYRO // 移动平均向量的长度（用于可调节的MMGYRO的最大值
      /* Moving Average ServoGimbal Signal Output */
      //#define MMSERVOGIMBAL                  // Active Output Moving Average Function for Servos Gimbal // 激活用于舵机云台的输出移动平均函数
      //#define MMSERVOGIMBALVECTORLENGHT 32   // Lenght of Moving Average Vector // 移动平均向量的长度

  /************************    Analog Reads     模拟读取     **********************************/
    /* if you want faster analog Reads, enable this. It may result in less accurate results, especially for more than one analog channel */
    /*如果你想更快的模拟读取，注释它。它可能会导致不准确的结果，特别是对多个模拟通道*/
    //#define FASTER_ANALOG_READS

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES          可选功能                        *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************************        Reset Baro altitude on arm     解锁后重置气压计高度    ********************/
  /* When unchecked a calibration of the baro altitude is preformed every time arming is activated */
  /** 如果未选中，则每次启用防护时都会进行大气压力高度校准。**/
  //#define ALTITUDE_RESET_ON_ARM

  /************************        Angele throttle correction    油门随着角度补偿      ********************/
  /* Automatically increase throttle based on the angle of the copter
     Original idea by Kraut Rob, first implementation HAdrian */
   /* 可以让你飞机倾斜的时候不要掉高度。根据角度增加油门,这是可以开下来的，但是要根据飞机不同来调节。免得角度一倾斜，油门窜得太高。
      最初构想***/

  //#define THROTTLE_ANGLE_CORRECTION 40
  
  /*** HEADFREE : the copter can be controled by an absolute stick orientation, whatever the yaw orientation ***/
  /*** 无头模式 : 起飞点和飞行器的连线将成为控制方向 ***/
  //#define HEADFREE
  
 /*************************        Advanced Headfree Mode             ********************/
 /*************************       高级 Headfree 无头模式             ********************/
 /* In Advanced Headfree mode when the copter is farther than ADV_HEADFREE_RANGE meters then 
    the  bearing between home and copter position will become the control direction 
    IF copter come closer than ADV_HEADFREE_RANGE meters, then the control direction freezed to the 
    bearing between home and copter at the point where it crosses the ADV_HEADFREE_RANGE meter distance
    first implementation by HAdrian, mods by EOSBandi
 */
/** 在高级无头模式下，当飞行机超过ADV_HEADFREE_RANGE定义的范围, 
    起飞点和飞行器的连线将成为控制方向, 当飞行器飞入ADV_HEADFREE_RANGE定义范围,那么控制方向将锁定为原点和飞行器飞入ADV_HEADFREE_RANGE
    范围内时位置的连线
    ***/

   //#define ADVANCED_HEADFREE      //Advanced headfree mode is enabled when this is uncommented
   //#define ADV_HEADFREE_RANGE 15  //Range where advanced headfree mode activated


  /************************        continuous gyro calibration    连续的陀螺仪校准   ********************/
  /* Gyrocalibration will be repeated if copter is moving during calibration. */
  /* 如果在校准过程中飞行器被移动，陀螺仪校准将会重复。 */
    //#define GYROCALIBRATIONFAILSAFE

  /************************        AP FlightMode        **********************************/
  /************************        AP飞行模式           **********************************/
  /*** FUNCTIONALITY TEMPORARY REMOVED ***/
    /* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
    /* 临时禁用GPS_HOLD_MODE（GPS保持模式），让移动摇杆时可以调整定点位置。*/
    //#define AP_MODE 40  // Create a deadspan for GPS.
        
  /************************   Assisted AcroTrainer    ************************************/
    /* Train Acro with auto recovery. Value set the point where ANGLE_MODE takes over.
       Remember to activate ANGLE_MODE first!...
       A Value on 200 will give a very distinct transfer */
    /************************    辅助特技练习器         ************************************/
    /* 在自动复原辅助下训练特技。该值设定ANGLE_MODE接管的点。
      记住首先激活ANGLE_MODE！...
      值为200将会给你一个很明显的转换 ***/
    //#define ACROTRAINER_MODE 200   // http://www.multiwii.com/forum/viewtopic.php?f=16&t=1944#p17437


  /********                          Failsafe settings                 ********************/
    /* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
       the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC is avaliable),
       PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
       for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed, 
       and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
/********                          失控保护设置                 ********************/
   /* 失控保护检查四个控制通道CH1-CH4的脉冲。如果脉冲丢失或低于985us（在这四个通道的任意一个上）
    失控保护程序就会启动。从失控保护检测到，再经过FAILSAFE_DELAY的时间，自稳模式就会开启（如果加速度或鸡腿柄可用），
    PITCH，ROLL和YAW被置中，油门设为FAILSAFE_THR0TTLE的值。你必须设定该值使下降速度在1m/s左右
    以获得最佳结果。该值取决于你的配置，总重量和一些其他参数。接下来，在FAILSAFE_OFF_DELAY之后，飞行器会被锁定，
    并且电机会停止。如果遥控脉冲在到达FAILSAFE_OFF_DELAY时间之前恢复，在很短的保护时间之后遥控就会恢复正常。 */
    //#define FAILSAFE                                // uncomment  to activate the failsafe function  // 解除注释以激活failsafe函数
    #define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example // 用于丢失信号之后失控保护激活之前的保护时间。1步=0.1秒 - 示例中为1秒
    #define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example // 用于电机停止前的着陆时间，以0.1秒为单位。1步=0.1秒 - 示例中为20秒
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case // (*) 用于降落的油门级别 - 可与MINTHROTTLE相关联 - 如本例所示
    
    #define FAILSAFE_DETECT_TRESHOLD  985


  /*****************                DFRobot LED RING    DFRobot LED 光环 *********************************/
    /* I2C DFRobot LED RING communication */ /* I2C DFRobot LED光环通讯 */
    //#define LED_RING

  /********************************    LED FLASHER    ***********************************/
  /********************************    LED闪光灯      ***********************************/
    //#define LED_FLASHER
    //#define LED_FLASHER_DDR DDRB
    //#define LED_FLASHER_PORT PORTB
    //#define LED_FLASHER_BIT PORTB4
    //#define LED_FLASHER_INVERT
    //#define LED_FLASHER_SEQUENCE        0b00000000      // leds OFF // leds关闭
    //#define LED_FLASHER_SEQUENCE_ARMED  0b00000101      // create double flashes // 创建双闪
    //#define LED_FLASHER_SEQUENCE_MAX    0b11111111      // full illumination // 全照明
    //#define LED_FLASHER_SEQUENCE_LOW    0b00000000      // no illumination // 无照明


  /*******************************    Landing lights    *********************************/
  /*******************************    着陆灯            *********************************/
  /* Landing lights
     Use an output pin to control landing lights.
     They can be switched automatically when used in conjunction
     with altitude data from a sonar unit. */
 /* 着陆灯
    使用一个输出针脚控制着陆灯。
    它与从声纳获得的高度数据结合时
    可以自动开关。 */
    //#define LANDING_LIGHTS_DDR DDRC
    //#define LANDING_LIGHTS_PORT PORTC
    //#define LANDING_LIGHTS_BIT PORTC0
    //#define LANDING_LIGHTS_INVERT

    /* altitude above ground (in cm) as reported by sonar */
    /* 依据声纳传来的数在地面之上的高度（以cm为单位） */
    //#define LANDING_LIGHTS_AUTO_ALTITUDE 50

    /* adopt the flasher pattern for landing light LEDs */
    /* 让闪光灯的样式应用于着陆灯LED */
    //#define LANDING_LIGHTS_ADOPT_LED_FLASHER_PATTERN

  /*************************    INFLIGHT ACC Calibration    *****************************/
    /* This will activate the ACC-Inflight calibration if unchecked */
    /*************************    飞行时加速度计校准           *****************************/
    /* 此项会激活加速度计飞行时校准 */
    //#define INFLIGHT_ACC_CALIBRATION

  /*******************************    OSD Switch    *************************************/
    // This adds a box that can be interpreted by OSD in activation status (to switch on/off the overlay for instance)
    /*******************************    OSD切换        *************************************/
    // 此项会添加一个可被OSD解读的激活状态的选框（比如说开关覆盖物）
  //#define OSD_SWITCH

  /**************************************************************************************/
  /***********************                  TX-related         **************************/
   /***********************              发射机-相关            **************************/
  /**************************************************************************************/

    /* introduce a deadband around the stick center
       Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
       /* 在摇杆中点周围引入一个死区（译者注：无作用控制区）
       必须大于零，如果你不需要在roll，pitch和yaw上的死区就注释掉它 */
    //#define DEADBAND 6

  /**************************************************************************************/
  /***********************                  GPS                **************************/
  /**************************************************************************************/

    /* ENable this for using GPS simulator (NMEA only)*/
    //#define GPS_SIMULATOR

    /* GPS using a SERIAL port
       if enabled, define here the Arduino Serial port number and the UART speed
       note: only the RX PIN is used in case of NMEA mode, the GPS is not configured by multiwii
       in NMEA mode the GPS must be configured to output GGA and RMC NMEA sentences (which is generally the default conf for most GPS devices)
       at least 5Hz update rate. uncomment the first line to select the GPS serial port of the arduino */
    /* GPS使用一个串口
     如果启用，在此定义Arduino串口号与UART速度
     注：如在NMEA模式只有RX针脚是被使用的，GPS不可被multiwii配置
     在NMEA模式下，GPS必须配置为输出GGA与RMC NMEA语句（在大部分GPS设备中通常为默认配置）
     至少为5Hz更新速率。解除第一行注释来选择用于GPS的arduino串口 */
       
    //#define GPS_SERIAL 2         // should be 2 for flyduino v2. It's the serial port number on arduino MEGA
                                   // must be 0 for PRO_MINI (ex GPS_PRO_MINI)
                                   // note: Now a GPS can share MSP on the same port. The only constrain is to not use it simultaneously, and use the same port speed.
                                   // 提示: 现在GPS可以共享同一端口的MSP。唯一的限制是不同时使用它，并使用相同的端口速度。

    // avoid using 115200 baud because with 16MHz arduino the 115200 baudrate have more than 2% speed error (57600 have 0.8% error)
    // 避免使用115200波特因为16MHz Arduino 115200波特率超过2%速度误差（57600有0.8%的误差）
    #define GPS_BAUD   57600       // GPS_BAUD will override SERIALx_COM_SPEED for the selected port

   /* GPS protocol 
       NMEA  - Standard NMEA protocol GGA, GSA and RMC  sentences are needed
       UBLOX - U-Blox binary protocol, use the ublox config file (u-blox-config.ublox.txt) from the source tree 
       MTK_BINARY16 and MTK_BINARY19 - MTK3329 chipset based GPS with DIYDrones binary firmware (v1.6 or v1.9)
       With UBLOX and MTK_BINARY you don't have to use GPS_FILTERING in multiwii code !!! */
    /* GPS协议
    NMEA  - 标准NMEA协议。需要GGA，GSA与RMC语句
    UBLOX - U-Blox二进制协议，使用来自源码树的ublox配置文件（u-blox-config.ublox.txt）
    MTK_BINARY16 与 MTK_BINARY19 - 基于MTK3329芯片的GPS，使用DIYDrones二进制固件（v1.6 或 v1.9）
    在使用UBLOX与MTK_BINARY时你不需要在multiwii代码中使用GPS_FILTERING!!! */
    
    //#define NMEA
    //#define UBLOX
    //#define MTK_BINARY16
    //#define MTK_BINARY19
    //#define INIT_MTK_GPS        // initialize MTK GPS for using selected speed, 5Hz update rate and GGA & RMC sentence or binary settings  // 初始化MTK GPS。使其使用选定的速度，5Hz更新速率与GGA & RMC语句或二进制的设置
    //#define VENUS8

    /* I2C GPS device made with an independant arduino + GPS device /// I2C GPS设备，使用一个独立的arduino + GPS设备制作   包含一些导航函数
       including some navigation functions
       contribution from EOSBandi   http://code.google.com/p/i2c-gps-nav/ 
       You have to use at least I2CGpsNav code r33 */
    /* all functionnalities allowed by SERIAL_GPS are now available for I2C_GPS: all relevant navigation computations are gathered in the main FC */

    //#define I2C_GPS

    // If your I2C GPS board has Sonar support enabled // 如果你的I2C GPS板有声纳支持
    //#define I2C_GPS_SONAR

    /* indicate a valid GPS fix with at least 5 satellites by flashing the LED  - Modified by MIS - Using stable LED (YELLOW on CRIUS AIO) led work as sat number indicator 
      - No GPS FIX -> LED blink at speed of incoming GPS frames
      - Fix and sat no. bellow 5 -> LED off
      - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ... */
      /* 通过LED闪烁表明GPS搜到了至少5颗有效的卫星 - 由MIS修改 - 使用常亮的LED（CRIUS AIO上为黄色）led作为星数指示器工作
    - GPS无定位 -> LED闪烁速度为收到GPS帧的速度
    - 定位并且星数小于5 -> LED关闭
    - 定位并且星数 >= 5 -> LED闪烁，闪一下表示5颗星，闪两下表示6颗星，三下表示7 ... */
    #define GPS_LED_INDICATOR

   //Enables the MSP_WP command set , which is used by WinGUI for displaying an setting up navigation
   // 家的地点(HOME position)会在每次解锁时重置，解除注释此项来禁用它（你可以通过校准陀螺仪来设置家的地点）
   // 启用MSP_WP命令，用于WinGUI显示与记录家与定点的位置
   //#define USE_MSP_WP

   // HOME position is reset at every arm, uncomment it to prohibit it (you can set home position with GyroCalibration)    
   // 家的地点(HOME position)会在每次解锁时重置，解除注释此项来禁用它（你可以通过校准陀螺仪来设置家的地点）
   //#define DONT_RESET_HOME_AT_ARM

/* GPS navigation can control the heading */
/* 允许GPS导航控制头部方向 */

// copter faces toward the navigation point, maghold must be enabled for it
// 飞行器面对着航点飞行，磁场保持必须为此开启
#define NAV_CONTROLS_HEADING       1    //(**)
// true - copter comes in with tail first
// true - 飞行器以尾部首先飞来
#define NAV_TAIL_FIRST             0    //(**)
// true - when copter arrives to home position it rotates it's head to takeoff direction
// true - 当飞行器到达家的位置时他会旋转至起飞时的角度
#define NAV_SET_TAKEOFF_HEADING    1    //(**)

/* Get your magnetic declination from here : http://magnetic-declination.com/
Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
Note the sign on declination it could be negative or positive (WEST or EAST)
Also note, that maqgnetic declination changes with time, so recheck your value every 3-6 months */
/* 从这里获取你的磁偏角：http://magnetic-declination.com/
   转换度+分至小数的角度，通过 ==> 度+分*(1/60)
   注意磁偏角的符号，它可为负或正（西或东） */
#define MAG_DECLINATION  4.02f   //(**)   (-1.55f   //(中国广西南宁市江南区))

// Adds a forward predictive filterig to compensate gps lag. Code based on Jason Short's lead filter implementation
// 添加向前预测滤波以补偿GPS延迟。代码基于Jason Short领导的滤波器实现
#define GPS_LEAD_FILTER               //(**)

// add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency comment out to disable
// use it with NMEA gps only 
// 添加5元素移动平均滤波器至GPS坐标，帮助消除GPS噪波但会增加延时，注释以禁用 
// 仅支持NMEA协议的GPS
//#define GPS_FILTERING                 //(**)

// if we are within this distance to a waypoint then we consider it reached (distance is in cm)
// 如果我们与航点在此距离以内，我们则认为已到达航点（以cm为单位）
#define GPS_WP_RADIUS              100      //(**)

// Safe WP distance, do not start mission if the first wp distance is larger than this number (in meters)
// Also aborts mission if the next waypoint distance is more than this number
// 安全的航路点的距离，如果第一个航路点的距离大于这个数，将不执行任务（单位：米）
// 同时，下一个航点间的距离大于这个数任务也会被终止（也就是两个航点间距离不能大于这个数）
#define SAFE_WP_DISTANCE           500      //(**)

//Maximu allowable navigation altitude (in meters) automatic altitude control will not go above this height
// 最大允许航行高度（米）高度自动控制不会超过这个高度
#define MAX_NAV_ALTITUDE           100     //(**)

// minimum speed when approach waypoint
//接近航点时的最小速度
#define NAV_SPEED_MIN              100    // cm/sec //(**)
// maximum speed to reach between waypoints
//最大速度达到之间的航点
#define NAV_SPEED_MAX              400    // cm/sec //(**)
// Slow down to zero when reaching waypoint (same as NAV_SPEED_MIN = 0)
// 到达航点时减速到零（与nav_speed_min = 0类似）
#define NAV_SLOW_NAV               0      //(**)
// Weight factor of the crosstrack error in navigation calculations (do not touch)
// 在导航计算的偏航错误的权重因子（别改）
#define CROSSTRACK_GAIN            .4     //(**)
// Maximum allowable banking than navigation outputs
//导航时的最大倾斜输出
#define NAV_BANK_MAX 3000                 //(**)

//Defines the RTH altitude. 0 means keep current alt during RTH (in meters)
//定义返回点高度。0是在返回点保持当时高度（米）
#define RTH_ALTITUDE               15        //(**)
//Wait to reach RTH alt before start moving to home (0-no, 1-yes)
//前往导航点前等待升高到预定高度（0-否，1-是）
#define WAIT_FOR_RTH_ALT           1         //(**)

//Navigation engine will takeover BARO mode control
//导航引擎接管气压定高模式工作
#define NAV_TAKEOVER_BARO          1         //(**)

//Throttle stick input will be ignored  (only in BARO)
//忽略油门杆的输入（只在气压定高模式）
#define IGNORE_THROTTLE            1         //(**)

//If FENCE DISTANCE is larger than 0 then copter will switch to RTH when it farther from home
//than the defined number in meters
//如果定义的范围大于0，飞行器将在超出此距离是自动切换到自动返航模式返回定义的返回点。
#define FENCE_DISTANCE      600

//This governs the descent speed during landing. 100 is equals approc 50cm/sec
//这参数控制自动降落模式的降落速度. 100表示下降速度为50厘米/秒
#define LAND_SPEED          100


    //#define ONLY_ALLOW_ARM_WITH_GPS_3DFIX      // Only allow FC arming if GPS has a 3D fix. // 限制飞控只能在GPS获取到三维定位数据后解锁

  /**************************************************************************************/
  /***********************        LCD/OLED - display settings       *********************/
  /**************************************************************************************/

    /* http://www.multiwii.com/wiki/index.php?title=Extra_features#LCD_.2F_OLED */

    /*****************************   The type of LCD     **********************************/
      /* choice of LCD attached for configuration and telemetry, see notes below */
      //#define LCD_DUMMY       // No Physical LCD attached.  With this & LCD_CONF defined, TX sticks still work to set gains, by watching LED blink.  
                                // 无物理LCD附加。通过定义此与LCD_CONF，发射机遥杆可用于设置增益，通过观察LED闪烁。
      //#define LCD_SERIAL3W    // Alex' initial variant with 3 wires, using rx-pin for transmission @9600 baud fixed
                                // Alex的初始变体使用3条导线，使用rx针脚进行传输@固定的9600波特率
      //#define LCD_TEXTSTAR    // SERIAL LCD: Cat's Whisker LCD_TEXTSTAR Module CW-LCD-02 (Which has 4 input keys for selecting menus)
                                // 串口LCD：Cat's Whisker品牌的LCD_TEXTSTAR模块CW-LCD-02（拥有4个输入按键用于选择菜单）
      //#define LCD_VT100       // SERIAL LCD: vt100 compatible terminal emulation (blueterm, putty, etc.)
                                // 串口LCD：vt100兼容终端仿真（blueterm，putty等）
      //#define LCD_TTY         // SERIAL LCD: useful to tweak parameters over cable with arduino IDE 'serial monitor'
                                // 串口LCD：用于通过线缆与arduino IDE“串口监视器”连接调整参数
      //#define LCD_ETPP        // I2C LCD: Eagle Tree Power Panel LCD, which is i2c (not serial)
                                // I2C LCD：Eagle Tree品牌的Power Panel LCD，使用i2c（非串口）
      //#define LCD_LCD03       // I2C LCD: LCD03, which is i2c
                                // I2C LCD：LCD03，使用i2c
      //#define LCD_LCD03S      // SERIAL LCD: LCD03 whit serial 9600 baud comunication enabled.
                                // 串口LCD：lcd03通过串口9600波特率通信。
      //#define OLED_I2C_128x64 // I2C LCD: OLED http://www.multiwii.com/forum/viewtopic.php?f=7&t=1350
      //#define OLED_DIGOLE     // I2C OLED from http://www.digole.com/index.php?productID=550

    /******************************   Display settings   ***********************************/
      #define LCD_SERIAL_PORT 0    // must be 0 on Pro Mini and single serial boards; Set to your choice on any Mega based board
                                   // 在Pro Mini以及其他单串口板上只能设为0，在任何基于Mega的板子上可设置为你的选择

      //#define SUPPRESS_OLED_I2C_128x64LOGO  // suppress display of OLED logo to save memory  // 禁用OLED logo显示来节省储存

    /* double font height for better readability. Reduces visible #lines by half.
     * The lower part of each page is accessible under the name of shifted keyboard letter :
     * 1 - ! , 2 - @ , 3 - # , 4 - $ , 5 - % , 6 - ^ , 7 - & , 8 - * , 9 - (
     * You must add both to your lcd.telemetry.* sequences
     */
    /**** 为获得更好的可读性，使用双倍字体高度。减少一半可见#行。
        每个页面的下半部分以按住shift的键盘文字作为名字：
        1 - ! , 2 - @ , 3 - # , 4 - $ , 5 - % , 6 - ^ , 7 - & , 8 - * , 9 - (
        你必须同时添加到你的lcd.遥测.*序列中
       ******/
      //#define DISPLAY_FONT_DSIZE //currently only aplicable for OLED_I2C_128x64 and OLED_DIGOLE //目前只能应用于OLED_I2C_128x64 OLED_DIGOLE

    /* style of display - AUTODETECTED via LCD_ setting - only activate to override defaults */
    /* 显示风格 - 通过LCD_ setting自动检测 - 仅在覆盖默认时激活 */
      //#define DISPLAY_2LINES
      //#define DISPLAY_MULTILINE
      //#define MULTILINE_PRE 2  // multiline configMenu # pref lines // 多行配置菜单#之前的行 
      //#define MULTILINE_POST 6 // multiline configMenu # post lines // 多行配置菜单#之前的行
      //#define DISPLAY_COLUMNS 16
    /********************************    Navigation     ***********************************/
    /* keys to navigate the LCD menu */
    /* 用来导航LCD配置菜单的按键 */
      #define LCD_MENU_PREV 'p'
      #define LCD_MENU_NEXT 'n'
      #define LCD_VALUE_UP 'u'
      #define LCD_VALUE_DOWN 'd'

      #define LCD_MENU_SAVE_EXIT 's'
      #define LCD_MENU_ABORT 'x'

  /**************************************************************************************/
  /***********************      LCD configuration menu         **************************/
  /**************************************************************************************/

    /* uncomment this line if you plan to use a LCD or OLED for tweaking parameters
    /* 如果你准备将LCD或OLED用于调整参数，那么解除本行注释
     * http://www.multiwii.com/wiki/index.php?title=Extra_features#Configuration_Menu */
      //#define LCD_CONF

    /* to include setting the aux switches for AUX1 -> AUX4 via LCD */
    /* 用于包含通过LCD进行AUX1 -> AUX4辅助开关切换的设置 */
      //#define LCD_CONF_AUX

    /* optional exclude some functionality - uncomment to suppress unwanted aux channel configuration options */
    /* 可选排除一些功能 - 解除注释以禁用一些不需要的遥测页面或通道设置 */
      //#define SUPPRESS_LCD_CONF_AUX2
      //#define SUPPRESS_LCD_CONF_AUX34

  /**************************************************************************************/
  /***********************      LCD       telemetry     遥测   **************************/
  /**************************************************************************************/

    /* to monitor system values (battery level, loop time etc. with LCD 
     * http://www.multiwii.com/wiki/index.php?title=LCD_Telemetry */

    /********************************    Activation   激活  ***********************************/
    //#define LCD_TELEMETRY

    /* to enable automatic hopping between a choice of telemetry pages uncomment this. */
    /* 在解除注释于此的一个遥测页面组合中启用自动跳转。 */
    //#define LCD_TELEMETRY_AUTO "123452679" // pages 1 to 9 in ascending order
    //#define LCD_TELEMETRY_AUTO  "212232425262729" // strong emphasis on page 2

    /* manual stepping sequence; first page of the sequence gets loaded at startup to allow non-interactive display */
    /* 手动步进序列；序列的第一页在启动时加载以允许无交互时显示 */
    //#define LCD_TELEMETRY_STEP "0123456789" // should contain a 0 to allow switching off.

    /* optional exclude some functionality - uncomment to suppress some unwanted telemetry pages */
    /*可选的默认项目的一些遥测页面-完整的可用功能列表见LCD.h */
    //#define SUPPRESS_TELEMETRY_PAGE_1
    //#define SUPPRESS_TELEMETRY_PAGE_2 // sensor readings // 传感器读数
    //#define SUPPRESS_TELEMETRY_PAGE_3 // checkboxitems // 复选框项
    //#define SUPPRESS_TELEMETRY_PAGE_4 // rx inputs // 遥控输入
    //#define SUPPRESS_TELEMETRY_PAGE_5 // servo&motor outputs // 舵机和电机输出
    //#define SUPPRESS_TELEMETRY_PAGE_6 // cells voltages // 电池电压
    //#define SUPPRESS_TELEMETRY_PAGE_7 // gps
    //#define SUPPRESS_TELEMETRY_PAGE_8 // alarms states // 告警状态
    //#define SUPPRESS_TELEMETRY_PAGE_9 // cycle & fails // 循环和失败
    //#define SUPPRESS_TELEMETRY_PAGE_R // reset // 重置

    /* optional override default items for some telemetry pages - for complete list of usable functions see LCD.h */
    /*可选的默认项目的一些遥测页面-完整的可用功能列表见LCD.h */
    //#define LCD_TELEMETRY_PAGE1 { output_V, output_mAh, }
    //#define LCD_TELEMETRY_PAGE2 { output_gyroX, output_gyroY, output_accZ, }
    //#define LCD_TELEMETRY_PAGE9 { output_fails, output_annex, output_debug0, output_debug3, }

  /********************************************************************/
  /****                             RSSI                           ****/
  /********************************************************************/
    //#define RX_RSSI
    //#define RX_RSSI_PIN A3
    //#define RX_RSSI_CHAN 8   //RSSI injection on selected channel (for PPM, Olrs, SBUS, etc.) (Starts at 0) 
                               //RSSI 注入指定的通道 (for PPM, Olrs, SBUS, etc.) (Starts at 0)

  /********************************************************************/
  /****                             TELEMETRY                      ****/
  /********************************************************************/
    // select one of the two protocols depending on your receiver
    //#define FRSKY_TELEMETRY           // used for FRSKY twoway receivers with telemetry (D-series like D8R-II or D8R-XP) 
                                      // VBAT, Baro, MAG, GPS and POWERMETER are helpful
                                      // VBAT_CELLS is optional for a forth screen on the display FLD-02
    //#define SPORT_TELEMETRY           // for FRSKY twoway receivers with S.PORT telemetry (S-series like X4R/X6R/X8R), not implemented yet - TO BE DONE

    // FRSKY common entries - valid for both protocols
    #define TELEMETRY_SERIAL 3        // change if required

    // FRSKY standard telemetry specific devices
    #define FRSKY_FLD02               // send only data specific for the FRSKY display FLD-02
    //#define OPENTX                    // send OpenTX specific data

    // FRSKY standard telemetry specific selections
    //#define COORDFORMAT_DECIMALMINUTES // uncomment to get the format DD°MM.mmmm for the coordinates - comment out to get the format DD.dddddd° for the coordinates 
    //#define KILOMETER_HOUR            // send speed in kilometers per hour instead of knots (default) - requested by OPENTX
    #define TELEMETRY_ALT_BARO        // send BARO based altitude, calibrated to 0 when arming, recommended if BARO available
    //#define TELEMETRY_ALT_GPS         // send GPS based altitude (altitude above see level), for FLD-02 don't use together with TELEMETRY_ALT_BARO
    #define TELEMETRY_COURSE_MAG      // send MAG based course/heading, recommended if MAG available, but FLD-02 does not display
    //#define TELEMETRY_COURSE_GPS      // send GPS based course/heading, don't use together with TELEMETRY_COURSE_MAG, FLD-02 does not display

    // S.PORT specific entries
    #define FRSKY_SPORT_A2_MAX 124    // A2 voltage is represented by a value in the range 0-255. A value of 16 results in 1.6V, 124 is 12.4V, etc

  /********************************************************************/
  /****                             Buzzer                         ****/
  /********************************************************************/
    //#define BUZZER
    //#define RCOPTIONSBEEP         // uncomment this if you want the buzzer to beep at any rcOptions change on channel Aux1 to Aux4
                                    // 如果你想在遥控选项在通道Aux1至Aux4改变时让蜂鸣器响起，解除注释此项
    //#define ARMEDTIMEWARNING 330  // (*) Trigger an alarm after a certain time of being armed [s] to save you lipo (if your TX does not have a countdown)
                                    // (*) 在解锁一段时间[s]后触发警报以保护锂电。（如果你的发射机没有倒计时）
    //#define PILOTLAMP             //Uncomment if you are using a X-Arcraft Pilot Lamp
                                    //如果你在使用X-Arcraft导航灯那么解除注释

  /********************************************************************/
  /****           battery voltage monitoring                       ****/
  /********************************************************************/
    /* for V BAT monitoring
       after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
       with R1=33k and R2=51k
       vbat = [0;1023]*16/VBATSCALE
       must be associated with #define BUZZER ! */
    /* 用于V BAT（电池电压）监控
        在电阻分压后，我们在模拟V_BAT针脚上应获得[0V;5V]->[0;1023]
        通过R1=33k和R2=51k
        vbat = [0;1023]*16/VBATSCALE
        必须与#define BUZZER结合! */

    //#define VBAT              // uncomment this line to activate the vbat code // 解除注释本行以激活vbat代码
    #define VBATSCALE       131 // (*) (**) change this value if readed Battery voltage is different than real voltage // (*) 如果读取到的电池电压与真实电压不同，修改该值
    #define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry // 12,6V满电标准电压 - 仅用于lcd.遥测
    #define VBATLEVEL_WARN1 107 // (*) (**) 10,7V // (*) (**) 10,7V
    #define VBATLEVEL_WARN2  99 // (*) (**) 9.9V // (*) (**) 9.9V
    #define VBATLEVEL_CRIT   93 // (*) (**) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered // (*) 9.3V - 临界情况：如果vbat持续低于该值，就会触发警报长响
    #define NO_VBAT          16 // Avoid beeping without any battery // (*) 避免在没有电池时响起
    #define VBAT_OFFSET       0 // offset in 0.1Volts, gets added to voltage value  - useful for zener diodes  //抵消0.1volts，加入有用的齐纳二极管的电压值

    /* for V BAT monitoring of individual cells
     * enable both VBAT and VBAT_CELLS
     */
    /* 对多个电池进行监控
      必须同时启用VBAT，VBAT_CELLS
    ***/
    //#define VBAT_CELLS
    #define VBAT_CELLS_NUM 0 // set this to the number of cells you monitor via analog pins // 设置连接在模拟阵脚pin上的电池数量
    #define VBAT_CELLS_PINS {A0, A1, A2, A3, A4, A5 } // set this to the sequence of analog pins // 将此设置为模拟引脚序列
    #define VBAT_CELLS_OFFSETS {0, 50, 83, 121, 149, 177 } // in 0.1 volts, gets added to voltage value  - useful for zener diodes 
    #define VBAT_CELLS_DIVS { 75, 122,  98, 18, 30, 37 } // divisor for proportional part according to resistors - larger value here gives smaller voltage

  /********************************************************************/
  /****           powermeter (battery capacity monitoring)         ****/
  /********************************************************************/

    /* enable monitoring of the power consumption from battery (think of mAh)
       allows to set alarm value in GUI or via LCD
      Full description and howto here http://www.multiwii.com/wiki/index.php?title=Powermeter
       Two options:
       1 - hard: - (uses hardware sensor, after configuration gives very good results)
       2 - soft: - (good results +-5% for plush and mystery ESCs @ 2S and 3S, not good with SuperSimple ESC)    */
    /* 启用电池能量消耗监控（以mAh考虑）
        允许在GUI中或通过LCD设置警戒值
        全部描述与操作方法请见此 http://www.multiwii.com/wiki/index.php?title=Powermeter
        有两个选项：
        1 - 硬件: - （使用硬件传感器，配置后将获得相当不错的结果）
        2 - 软件: - （使用plush与mystery电调可获得+-5%的料号结果，使用SuperSimple电调结果不佳）  ****/
    //#define POWERMETER_SOFT
    //#define POWERMETER_HARD
    #define PSENSORNULL 510 /* (*) hard only: set to analogRead() value for zero current; for I=0A my sensor 
                                   gives 1/2 Vss; that is approx 2.49Volt; */
                                   // (*) 设置0电流时analogRead()的值；I=0A时，我的传感器得到1/2 Vss；约为2.49伏；
    #define PINT2mA 132     /* (*) hard: one integer step on arduino analog translates to mA (example 4.9 / 37 * 1000) ; // (*) 用于遥测显示：一个用在arduino模拟转换为mA时的整数（例4.9 / 37 * 100
                                   soft: use fictional value, start with 100.
                                   for hard and soft: larger PINT2mA will get you larger value for power (mAh equivalent) */
    //#define WATTS // compute and display the actual watts (=Volt*Ampere) consumed - requires both POWERMETER_HARD and VBAT  // 计算并显示实际瓦（=伏特×安培）需要powermeter_hard和VBAT

  /********************************************************************/
  /****           altitude hold          高度保持                   ****/
  /********************************************************************/

    /* defines the neutral zone of throttle stick during altitude hold, default setting is
       +/-50 uncommend and change the value below if you want to change it. */
    /** 定义高度保持期间油门杆的空档区域，默认设置为+/-50取消注释，如果要更改该值，请更改下面的值。**/
    #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE    50
    //#define ALT_HOLD_THROTTLE_MIDPOINT        1500  // in us    - if uncommented, this value is used in ALT_HOLD for throttle stick middle point instead of initialThrottleHold parameter.
                                                     // 如果未注释，则在油门杆中间点的alt_hold中使用此值，而不是使用initialThrottlehold参数。


    /* uncomment to disable the altitude hold feature.
     * This is useful if all of the following apply
     * + you have a baro
     * + want altitude readout and/or variometer
     * + do not use altitude hold feature
     * + want to save memory space */
    /* 解除注释以禁用高度保持特性。
         此项可用于所有下列应用
         + 你有一个气压传感器
         + 想要高度值输出
         + 不需要使用高度保持特性
         + 想要节省储存空间
     ****/
    //#define SUPPRESS_BARO_ALTHOLD

  /********************************************************************/
  /****         altitude variometer     高度爬升率测定器    (高度仪)    ****/
  /********************************************************************/

    /* enable to get audio feedback upon rising/falling copter/plane.
     * Requires a working baro.
     * For now, Output gets sent to an enabled vt100 terminal program over the serial line.
     * choice of two methods (enable either one or both)
     * method 1 : use short term movement from baro ( bigger code size)
     * method 2 : use long term observation of altitude from baro (smaller code size)
     */
     /* 启用以获得来自上升/下降中的飞行器/飞机的声频反馈。
        需要工作中的气压计。
        目前，输出会通过串行线发送至启用中的vt100终端程序。
        有两种方式可选（启用其中一个或同时启用）
        方式1：使用来自气压计的短期移动（更大的代码尺寸）
        方式2：使用来自气压计的长期高度观察（更小的代码尺寸）
       */
    //#define VARIOMETER 12            // possible values: 12 = methods 1 & 2 ; 1 = method 1 ; 2 = method 2
    //#define SUPPRESS_VARIOMETER_UP   // if no signaling for up movement is desired
    //#define SUPPRESS_VARIOMETER_DOWN // if no signaling for down movement is desired
    //#define VARIOMETER_SINGLE_TONE   // use only one tone (BEL); neccessary for non-patched vt100 terminals

  /********************************************************************/
  /****           board naming                                     ****/
  /********************************************************************/

    /*
     * this name is displayed together with the MultiWii version number
     * upon powerup on the LCD.
     * If you are without a DISPLAYD then You may enable LCD_TTY and
     * use arduino IDE's serial monitor to view the info.
     *
     * You must preserve the format of this string!
     * It must be 16 characters total,
     * The last 4 characters will be overwritten with the version number.
     */
    /*
        这个名字会与MultiWii版本号共同显示
        在打开电源时显示在LCD上。
        如果你没有显示设备那么你可以启用LCD_TTY并
        使用arduino IDE的串口监控器来查看此信息。
        你必须保持此处文本的格式！
        它必须总共有16个字母，
        最后4个字母将会被版本号覆盖。
       */
    #define BOARD_NAME "MultiWii   V-.--"
    //                  123456789.123456

  /*************      Support multiple configuration profiles in EEPROM     ************/
    //#define MULTIPLE_CONFIGURATION_PROFILES

  /*************      do no reset constants when change of flashed program is detected ***********/
  /*************      在EEPROM中支持多个配置参数文件      ************/
    #define NO_FLASH_CHECK

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/****************        第 7 部分 - 调试 & 开发者                                 **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

  #define VBAT_PRESCALER 16 // set this to 8 if vbatscale would exceed 255  //设置为16，如果vbatscale将超过255

  /**************************************************************************************/
  /********   special ESC with extended range [0-2000] microseconds  ********************/
  /********   使用扩展范围[0-2000]微秒的特殊电调              ********************/
  /**************************************************************************************/
    //#define EXT_MOTOR_RANGE // using this with wii-esc requires to change MINCOMMAND to 1008 for promini and mega
    // 在用场效应管驱动空心杯时必须注释它，否则电机上电就转

  /**************************************************************************************/
  /********  brushed ESC ***   刷电调   **************************************************/
  /**************************************************************************************/
    // for 328p proc
    //#define EXT_MOTOR_32KHZ
    //#define EXT_MOTOR_4KHZ
    //#define EXT_MOTOR_1KHZ
  
    // for 32u4 proc
    //#define EXT_MOTOR_64KHZ
    //#define EXT_MOTOR_32KHZ
    //#define EXT_MOTOR_16KHZ
    //#define EXT_MOTOR_8KHZ

  /**************************************************************************************/
  /***********************     motor, servo and other presets     ***********************/
  /***********************     电机，舵机和其他的预置             ***********************/
  /**************************************************************************************/
  /* 当油门命令在低位时电机将不会旋转
     这是立即停止电机的替代方案 */
    /* motors will not spin when the throttle command is in low position
       this is an alternative method to stop immediately the motors */
    //#define MOTOR_STOP

    /* some radios have not a neutral point centered on 1500. can be changed here */
    /* 一些遥控器的中立点不是1500。可以在此修改 */
    #define MIDRC 1500

  /***********************         Servo Refreshrates      舵机刷新率    **********************/
    /* Default 50Hz Servo refresh rate*/   /* 默认50Hz舵机刷新率 */
    #define SERVO_RFR_50HZ

    /* up to 160Hz servo refreshrate .. works with the most analog servos*/
    /* 升至160Hz舵机刷新率 .. 用于多数模拟舵机 */
    //#define SERVO_RFR_160HZ

    /* up to 300Hz refreshrate it is as fast as possible (100-300Hz depending on the cound of used servos and the servos state).
       for use with digital servos
       dont use it with analog servos! thay may get damage. (some will work but be careful) */
       /* 升至300Hz刷新率，它越快越好（100-300Hz取决于使用的舵机和舵机状态）。
        用于数字舵机
        不要用于模拟舵机！它们可能遭到破坏。（一些可以使用，但请非常小心） */
    //#define SERVO_RFR_300HZ
    
  /***********************             HW PWM Servos             ***********************/ 
    /* HW PWM Servo outputs for Arduino Mega.. moves:
     硬件PWM舵机输出用于Arduino Mega..移动至：*
      Pitch   = pin 44
      Roll    = pin 45
      CamTrig = pin 46
      SERVO4  = pin 11 (aileron left for fixed wing or TRI YAW SERVO)
      SERVO5  = pin 12 (aileron right for fixed wing)
      SERVO6  = pin 6   (rudder for fixed wing)
      SERVO7  = pin 7   (elevator for fixed wing)
      SERVO8  = pin 8   (motor for fixed wing)       */ 

 /*此选项禁用其他用于舵机的软件PWM - 仅有五个硬件控制舵机可用      */ 
    #define MEGA_HW_PWM_SERVOS
 
    /* HW PWM Servo outputs for 32u4 NanoWii, MicroWii etc. - works with either the variable SERVO_RFR_RATE or
     * one of the 3 fixed servo.refresh.rates *
     * Tested only for heli_120, i.e. 1 motor + 4 servos, moves..
     * motor[0] = motor       = pin  6
     * servo[3] = nick  servo = pin 11
     * servo[4] = left  servo = pin 10
     * servo[5] = yaw   servo = pin  5
     * servo[6]  = right servo= pin  9
     */
    //#define A32U4_4_HW_PWM_SERVOS

    #define SERVO_RFR_RATE  50    // In Hz, you can set it from 20 to 400Hz, used only in HW PWM mode for mega and 32u4
    //#define SERVO_PIN5_RFR_RATE  200    // separate yaw pwm rate.
                                          // In Hz, you can set it from 20 to 400Hz, used only in HW PWM mode for 32u4


  /********************************************************************/
  /****           Memory savings        节约记忆体空间              ****/
  /********************************************************************/

    /* options to counter the general shortage of both flash and ram memory, like with leonardo m32u4 and others */
    /* 针对Flash和RAM内存的普遍短缺 leonardo m32u4 and others */

    /**** suppress handling of serial commands.***
     * This does _not_ affect handling of RXserial, Spektrum or GPS. Those will not be affected and still work the same.
     * Enable either one or both of the following options  */
    /**** 可以通过禁用串口命令处理来实现。***
     * 它_不会_对RXserial，Spektrum，GPS的处理产生影响。这些不会受到影响，仍可以照常工作。
     * 启用下列选项中其中一项或两项  */

      /* Remove handling of all commands of the New MultiWii Serial Protocol.
       * This will disable use of the GUI, winGUI, android apps and any other program that makes use of the MSP.
       * You must find another way (like LCD_CONF) to tune the parameters or live with the defaults.
       * If you run a LCD/OLED via i2c or serial/Bluetooth, this is safe to use */
       /* 移除所有新MultiWii串行协议命令的处理。
          这将会禁用GUI，winGUI，android应用以及其他所有使用MSP的程序。
          你必须找到其他调试参数的方法（如LCD_CONF）或保持默认。
          如果你是通过i2c或串口/蓝牙使用LCD/OLED，可以放心使用 */
      //#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes // 节省约2700字节

      /* Remove handling of other serial commands.
       * This includes navigating via serial the lcd.configuration menu, lcd.telemetry and permanent.log .
       * Navigating via stick inputs on tx is not affected and will work the same.  */
      /* 移除其他串行命令处理。
         包含通过串口操作lcd.配置菜单，lcd.遥测与永久.日志。
         通过在发射机上摇杆输入进行操作不会受到影响，操作起来是一样的。  */
      //#define SUPPRESS_OTHER_SERIAL_COMMANDS // saves  approx 0 to 100 bytes, depending on features enabled // 节省约0至100字节，取决于启用的特性

    /**** suppress keeping the defaults for initial setup and reset in the code.
     * This requires a manual initial setup of the PIDs etc. or load and write from defaults.mwi;
     * reset in GUI will not work on PIDs
     */
    /**** 保证代码中无初始设置和复位的缺陷。
       这需要一个手动初始设置的PID等手动写defaults.mwi
     ****/
    //#define SUPPRESS_DEFAULTS_FROM_GUI
    
    //#define DISABLE_SETTINGS_TAB  // Saves ~400bytes on ProMini

  /********************************************************************/
  /****           diagnostics    诊断                              ****/
  /********************************************************************/

    /* to log values like max loop time and others to come
       logging values are visible via LCD config
       set to 1, enable 'R' option to reset values, max current, max altitude
       set to 2, adds min/max cycleTimes
       set to 3, adds additional powerconsumption on a per motor basis (this uses the big array and is a memory hog, if POWERMETER <> PM_SOFT) */
    /* 记录像最大周期时间与其他可能的值
        记录值可通过LCD配置看到
        设为1，启用'R'选项来重置值，最大电流，最大高度
        设为2，添加最大/最小周期时间
        设为3，以每个电机为单位添加额外的功耗（它使用一个很大的数组并且很吃储存，如果POWERMETER <> PM_SOFT） */
    //#define LOG_VALUES 1

    /* Permanent logging to eeprom - survives (most) upgrades and parameter resets.
     * used to track number of flights etc. over lifetime of controller board.
     * Writes to end of eeprom - should not conflict with stored parameters yet.
     * Logged values: accumulated lifetime, #powercycle/reset/initialize events, #arm events, #disarm events, last armedTime,
     *                #failsafe@disarm, #i2c_errs@disarm
     * Enable one or more options to show the log
     */
    /* 永久记录至eeprom - 可在（多数）升级与参数重置中保留下来。
       常用于追踪控制板生命周期中的飞行次数等。
       写入至eeprom末端 - 不应与已储存的参数冲突。
       记录的值：累积的生存时间，#重启/重置/初始化事件，#解锁事件，#锁定事件，最后解锁时间，
                      #失控保护@锁定，#i2c_errs@锁定
       设置你的mcu的eeprom的尺寸以激活：promini 328p：1023；2560：4095。
       启用一项或更多选项以显示记录
      */
    //#define LOG_PERMANENT
    //#define LOG_PERMANENT_SHOW_AT_STARTUP // enable to display log at startup
    //#define LOG_PERMANENT_SHOW_AT_L // enable to display log when receiving 'L'
    //#define LOG_PERMANENT_SHOW_AFTER_CONFIG // enable to display log after exiting LCD config menu
    //#define LOG_PERMANENT_SERVICE_LIFETIME 36000 // in seconds; service alert at startup after 10 hours of armed time

    /* to add debugging code
       not needed and not recommended for normal operation
       will add extra code that may slow down the main loop or make copter non-flyable */
       /* 添加调试代码
        不需要并且也不推荐在平常运行时开启
        将会额外添加代码，可能会使主循环变慢或使飞行器不可飞行 */
    //#define DEBUG
    //#define DEBUG_FREE // will add 'F' command to show free memory

    /* Use this to trigger LCD configuration without a TX - only for debugging - do NOT fly with this activated */
    /* 使用此项在没有发射机时触发LCD配置 - 仅用于调试 - 不要在此项激活的情况下飞行 */
    //#define LCD_CONF_DEBUG

    /* Use this to trigger telemetry without a TX - only for debugging - do NOT fly with this activated */
    /* 使用此项在没有发射机时触发遥测 - 仅用于调试 - 不要在此项激活的情况下飞行 */
    //#define LCD_TELEMETRY_DEBUG    //This form rolls between all screens, LCD_TELEMETRY_AUTO must also be defined.
    //#define LCD_TELEMETRY_DEBUG 6  //This form stays on the screen specified.

    /* Enable string transmissions from copter to GUI */
    /* 启用从飞行器到GUI的字符串传送 */
    //#define DEBUGMSG


  /********************************************************************/
  /****           ESCs calibration     电调校准                     ****/
  /********************************************************************/

    /* to calibrate all ESCs connected to MWii at the same time (useful to avoid unplugging/re-plugging each ESC)
       Warning: this creates a special version of MultiWii Code
       You cannot fly with this special version. It is only to be used for calibrating ESCs
       Read How To at http://code.google.com/p/multiwii/wiki/ESCsCalibration */
       /* 同时校准所有连接到MWii的电调（可以避免来回连接每一个电调）
         警告：这将产生一个特别版本的MultiWii代码
        这个特殊的版本是不可以用来飞行的。它只可以用来校准电调 ***/
    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000
    //#define ESC_CALIB_CANNOT_FLY  // uncomment to activate // 解除注释激活此项，千万注意，校准电调时请拆卸下你的螺旋桨！

  /****           internal frequencies                             ****/
    /****           内部频率                                         ****/
    /* frequenies for rare cyclic actions in the main loop, depend on cycle time
       time base is main loop cycle time - a value of 6 means to trigger the action every 6th run through the main loop
       example: with cycle time of approx 3ms, do action every 6*3ms=18ms
       value must be [1; 65535] */
     /* 在主循环中的稀有循环操作的频率，取决于周期时间
        时间基数为主循环周期时间 - 值为6意味着每六个主循环触发一次操作
        示例：周期时间大约在3ms，执行操作就在每 6*3ms=18ms
        取值范围 [1; 65535] ****/
    #define LCD_TELEMETRY_FREQ 23       // to send telemetry data over serial 23 <=> 60ms <=> 16Hz (only sending interlaced, so 8Hz update rate) // 通过串口发送遥测数据 23 <=> 60ms <=> 16Hz （只发送隔行数据，8Hz上传速率）
    #define LCD_TELEMETRY_AUTO_FREQ  967// to step to next telemetry page 967 <=> 3s // 翻到下一个遥测页面 967 <=> 3s
    #define PSENSOR_SMOOTH 16           // len of averaging vector for smoothing the PSENSOR readings; should be power of 2; set to 1 to disable
    #define VBAT_SMOOTH 16              // len of averaging vector for smoothing the VBAT readings; should be power of 2; set to 1 to disable
    #define RSSI_SMOOTH 16              // len of averaging vector for smoothing the RSSI readings; should be power of 2; set to 1 to disable

  /********************************************************************/
  /****           Dynamic Motor/Prop Balancing   桨/马达动平衡      ****/
  /********************************************************************/
  /*                   !!! No Fly Mode !!!                            */ 

    //#define DYNBALANCE   // (**) Dynamic balancing controlled from Gui // (**) 用gui调整动平衡

  /********************************************************************/
  /****           Regression testing            回归测试           ****/
  /********************************************************************/

    /* for development only:
       to allow for easier and reproducable config sets for test compiling, different sets of config parameters are kept
       together. This is meant to help detecting compile time errors for various features in a coordinated way.
       It is not meant to produce your flying firmware
       To use:
       - do not set any options in config.h,
       - enable with #define COPTERTEST 1, then compile
       - if possible, check for the size
       - repeat with other values of 2, 3, 4 etc.
        */
        /* 只用作开发用途:
          考虑到测试编译时，不同的config参数是保持在一起的，所以可以更简单地重复测试config设置，
          它的意义是可以帮助检测编译时的错误，让多种不同的特性以协调的方式运作。
          这并不是用来制作你自己的飞行固件的。
          使用方法：
        - 不要在config.h中做任何设置，
        - 启用#define COPTERTEST 1，然后编译
        - 如果可能的话，检查程序大小
        - 重复测试其他值2, 3, 4等。
         */
    //#define COPTERTEST 1

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  8 - DEPRECATED             不推荐使用                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /* these features will be removed in the unforseeable future. Do not build new products or
   * functionality based on such features. The default for all such features is OFF.
   */
  /**** 这些功能将在未来被移除的。不再更新
    基于这样的特点功能。所有这些功能默认是关闭的。
    ****/

  /**************************    WMP power pin     *******************************/
  /**************************    WMP的电源引脚    *******************************/
  //#define D12_POWER      // Use D12 on PROMINI to power sensors. Will disable servo[4] on D12
  /* disable use of the POWER PIN (allready done if the option RCAUXPIN12 is selected) */
  #define DISABLE_POWER_PIN

/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/****                            可配置参数结束                                               ****/
/*************************************************************************************************/

#endif /* CONFIG_H_ */

