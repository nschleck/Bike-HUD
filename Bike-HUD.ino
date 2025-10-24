/*
  This code aims to create a mini heads-up display (HUD) to provide data while biking -- speed, heading, and more.
  
  This code is built to run specifically on a Waveshare ESP32-S3-Touch 1.28" Round (240 x 240) Display
    -> wiki: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.28
    -> upload settings: ESP32S3 Dev Module; Flash Size 16 MB; Partition 16M Flash, 3MB APP, PSRAM QSPI

  Library dependencies:
    <TFT_eSPI>    -> display driver (** TODO must be configured for GC9A01 **)
    "CST816S"     -> touch driver (included as a project file)
    <LVGL>        -> graphics library (** version 8.3.11 ? **)

    <WiFi>        -> ...WiFi, of course
    <ESP32Time>   -> RTC
    <MadgwickAHRS>-> AHRS algorithm implementation; MadgwickAHRS from https://github.com/arduino-libraries/MadgwickAHRS

  Roadmap:
    DONE get display and touch functions running
    DONE implement AHRS
    DONE implement LVGL graphics
    TODO implement compass indicator
    TODO implement slope indicator
    TODO implement RTC

    TODO implement speed measurement
    TODO implement distance measurement
    
    TODO implement trip logger
      -> distance travelled
      -> top speed
      -> elapsed time

*/

//TODO figure out how to subfolder rider files

//~~~~~~~~~ Library Includes ~~~~~~~~~//
  #include <TFT_eSPI.h>           // display driver
  #include "CST816S.h"            // touch driver
  #include <lvgl.h>               // graphics library
  #include "lv_conf.h"            // LVGL config file

  #include <Math.h>
  #include "SensorQMI8658.hpp"    // 6-axis IMU
  #include <MadgwickAHRS.h>       // AHRS algorithm

//~~~~~~~~~~ Pin Definitions ~~~~~~~~~//
  #define USE_I2C
  #define BAT_ADC_PIN 1
  #define SENSOR_SDA  6
  #define SENSOR_SCL  7
  const int backlightPin = 2;

//~~~~~~~~~~ Display Variables ~~~~~~~~~//
  static const uint16_t screenWidth  = 240;
  static const uint16_t screenHeight = 240;
  static const uint16_t screenRadius = screenWidth/2;

  TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
  CST816S touch(6, 7, 13, 5);	// sda, scl, rst, irq

//~~~~~~~~~~ IMU  Variables ~~~~~~~~~~~~//
  SensorQMI8658 qmi;
  IMUdata acc;
  IMUdata gyr;
  int IMUPeriod = 40; // total milliseconds for IMU data refresh
  int IMU_timestep;
  unsigned long IMUPeriodStart;

  Madgwick filter;
  //uint32_t microsPerReading = 1000000 / 25;
  float AHRS_frequency = 25;
  float roll, pitch, heading;

//~~~~~~~~~~ LVGL Declarations ~~~~~~~~~//
  // declare display buffers
    static lv_disp_draw_buf_t draw_buf;
    static lv_disp_drv_t disp_drv;
    static lv_color_t buf[ screenWidth * screenHeight / 10 ];

  // Declare theme fonts
    LV_FONT_DECLARE(netto_black_72); // 0-9 only

  // Define Theme Colors
    lv_color_t white =      lv_color_hex(0xFFFFFF);
    lv_color_t off_white =  lv_color_hex(0xE7E7DA);
    lv_color_t grey =       lv_color_hex(0x707070);
    lv_color_t black =      lv_color_hex(0x000000);
    
    lv_color_t yellow     = lv_color_hex(0xF39F2B);
    lv_color_t periwinkle = lv_color_hex(0x926EED);
    lv_color_t taupe =      lv_color_hex(0x483D3F);
  
  // declare GUI objects
    lv_obj_t * screen; // "home" screen

    static lv_obj_t * heading_label;
    static lv_obj_t * pitch_label;
    static lv_obj_t * roll_label;
    static lv_obj_t * AHRS_dot;
    

//~~~~~~~~~~ Timing Variables ~~~~~~~~~~~//
  unsigned long currentMillis;

void setup() {
  pinMode(backlightPin, OUTPUT);
  Serial.begin(115200);

  initPSRAM();

  // Set up display and touch
    lv_init();
    tft.begin();
    touch.begin();
    printLVGLVersion();
    initLVGLDisplay();
    initLVGLInput();
  
  // begin AHRS
    initIMU();
    filter.begin(AHRS_frequency);

  // Set up LVGL Theme and screen
    lv_disp_t * display = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(display,                 
                                            yellow,   /* Primary and secondary palette */
                                            periwinkle,
                                            true,        /* True = Dark theme,  False = light theme. */
                                            &lv_font_montserrat_28);

    screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, screenWidth, screenHeight);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_scr_load(screen);

  // TEMP -- Create GUI objects
    lv_obj_t * tabview = lv_tabview_create(screen, LV_DIR_TOP, 0);
    lv_obj_t * tab1 = createTab(tabview);
    lv_obj_t * tab2 = createTab(tabview);
    lv_obj_t * tab3 = createTab(tabview);

    // tab 1
      lv_obj_t * speed_label = lv_label_create(tab1);
      lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, 0);
      lv_obj_set_style_text_font(speed_label, &netto_black_72, 0);
      lv_obj_set_style_text_align(speed_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
      lv_obj_set_style_text_opa(speed_label, LV_OPA_90, NULL);
      lv_obj_set_style_text_line_space(speed_label, 10, NULL);
      lv_label_set_text(speed_label, "10\nMPH");

    // tab 2
      lv_obj_t * spinner_t2 = lv_spinner_create(tab2, 4000, 60);
      lv_obj_set_size(spinner_t2, 230, 230);
      lv_obj_center(spinner_t2); 
      lv_obj_set_style_arc_color(spinner_t2, periwinkle, LV_PART_INDICATOR);

      heading_label = lv_label_create(tab2);
      pitch_label = lv_label_create(tab2);
      roll_label = lv_label_create(tab2);
      lv_obj_align(heading_label, LV_ALIGN_CENTER, 0, -20);
      lv_obj_align(pitch_label, LV_ALIGN_CENTER, 0, 0);
      lv_obj_align(roll_label, LV_ALIGN_CENTER, 0, 20);
      lv_obj_set_style_text_font(heading_label, &lv_font_montserrat_20, NULL);
      lv_obj_set_style_text_font(pitch_label, &lv_font_montserrat_20, NULL);
      lv_obj_set_style_text_font(roll_label, &lv_font_montserrat_20, NULL);

      AHRS_dot = lv_btn_create(tab2);
      lv_obj_center(AHRS_dot);
      lv_obj_set_size(AHRS_dot, 30, 30);
      lv_obj_set_style_radius(AHRS_dot, 15, 0);

    
  //~~~~~~~~~ Finish Setup ~~~~~~~~~~~//
    Serial.println("Setup done");
    IMUPeriodStart = millis();
}

void loop() {
  lv_timer_handler();
  delay(5);
  currentMillis = millis();

  // Handle IMU Data
  if (((currentMillis - IMUPeriodStart) > IMUPeriod) && qmi.getDataReady()) { 
    updateIMU();
  }
}

/* ********************** FUNCTIONS ************************ */
  //~~~~~~~~~~~ Display/Touch Functions ~~~~~~~~~~~~~~//
    void flush_display( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p ){
        uint32_t w = ( area->x2 - area->x1 + 1 );
        uint32_t h = ( area->y2 - area->y1 + 1 );

        tft.startWrite();
        tft.setAddrWindow( area->x1, area->y1, w, h );
        tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
        tft.endWrite();

        lv_disp_flush_ready( disp_drv );
    }

    void read_touchpad( lv_indev_drv_t * indev_drv, lv_indev_data_t * data ){
        bool touched = touch.available();
        // touch.read_touch();
        if(!touched){
        // if( 0!=touch.data.points )
            data->state = LV_INDEV_STATE_REL;
        }else{
            data->state = LV_INDEV_STATE_PR;
            data->point.x = touch.data.x;
            data->point.y = touch.data.y;
        }
    }
  //~~~~~~~~~~~ IMU functions ~~~~~~~~~~~~~~~~~~~~~~~~//
    void initIMU(){
      //beginPower();
      if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
          delay(1000);
        }
      }

      Serial.print("Device ID:");
      Serial.println(qmi.getChipID(), HEX);

      if (qmi.selfTestAccel()) {
        Serial.println("Accelerometer self-test successful");
      } else {
        Serial.println("Accelerometer self-test failed!");
      }

      if (qmi.selfTestGyro()) {
        Serial.println("Gyroscope self-test successful");
      } else {
        Serial.println("Gyroscope self-test failed!");
      }

      qmi.configAccelerometer(
            /*
              * ACC_RANGE_2G
              * ACC_RANGE_4G
              * ACC_RANGE_8G
              * ACC_RANGE_16G
              * */
            SensorQMI8658::ACC_RANGE_2G,
            /*
              * ACC_ODR_1000H
              * ACC_ODR_500Hz
              * ACC_ODR_250Hz
              * ACC_ODR_125Hz
              * ACC_ODR_62_5Hz
              * ACC_ODR_31_25Hz
              * ACC_ODR_LOWPOWER_128Hz
              * ACC_ODR_LOWPOWER_21Hz
              * ACC_ODR_LOWPOWER_11Hz
              * ACC_ODR_LOWPOWER_3H
            * */
            SensorQMI8658::ACC_ODR_1000Hz,
            /*
            *  LPF_MODE_0     //2.66% of ODR
            *  LPF_MODE_1     //3.63% of ODR
            *  LPF_MODE_2     //5.39% of ODR
            *  LPF_MODE_3     //13.37% of ODR
            *  LPF_OFF        // OFF Low-Pass Fitter
            * */
            SensorQMI8658::LPF_MODE_0);

      qmi.enableAccelerometer();
      qmi.configGyroscope(
          /*
          * GYR_RANGE_16DPS
          * GYR_RANGE_32DPS
          * GYR_RANGE_64DPS
          * GYR_RANGE_128DPS
          * GYR_RANGE_256DPS
          * GYR_RANGE_512DPS
          * GYR_RANGE_1024DPS
          * */
          SensorQMI8658::GYR_RANGE_512DPS,
          /*
          * GYR_ODR_7174_4Hz
          * GYR_ODR_3587_2Hz
          * GYR_ODR_1793_6Hz
          * GYR_ODR_896_8Hz
          * GYR_ODR_448_4Hz
          * GYR_ODR_224_2Hz
          * GYR_ODR_112_1Hz
          * GYR_ODR_56_05Hz
          * GYR_ODR_28_025H
          * */
          SensorQMI8658::GYR_ODR_896_8Hz,
          /*
          *  LPF_MODE_0     //2.66% of ODR
          *  LPF_MODE_1     //3.63% of ODR
          *  LPF_MODE_2     //5.39% of ODR
          *  LPF_MODE_3     //13.37% of ODR
          *  LPF_OFF        // OFF Low-Pass Fitter
          * */
          SensorQMI8658::LPF_MODE_3);
      qmi.enableGyroscope();
      qmi.dumpCtrlRegister();

    }

    void updateIMU(){
      // Serial.print(qmi.getTimestamp());
      // qmi.getTemperature_C();
      IMU_timestep = currentMillis - IMUPeriodStart;
      if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
        // update_accel_label("x: ", x_accel_label, acc.x);
        // update_accel_label("y: ", y_accel_label, acc.y);
        // update_accel_label("z: ", z_accel_label, acc.z);
        // update_IMU_dot_inertial(acc.x, acc.y, IMU_timestep);
      }
      qmi.getGyroscope(gyr.x, gyr.y, gyr.z);

      // Madgwick Algo
        filter.updateIMU(gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z);
        roll = filter.getRoll();
        if (roll < 0){
          roll += 180;
        } else {
          roll -=180;
        }
        pitch = filter.getPitch();
        heading = filter.getYaw();
        Serial.print("heading:"); Serial.print(heading); Serial.print(",");
        Serial.print("pitch:"); Serial.print(pitch); Serial.print(",");
        Serial.print("roll:"); Serial.print(roll); Serial.println();
        update_AHRS_label("H: ", heading_label, heading);
        update_AHRS_label("P: ", pitch_label, pitch);
        update_AHRS_label("R: ", roll_label, roll);
        update_AHRS_dot(heading, pitch, roll);

      IMUPeriodStart = millis();
    }

  //~~~~~~~~~~~ LVGL functions ~~~~~~~~~~~~~~~~~~~~//
    //************ Initialization **************//
      void printLVGLVersion(){
        String LVGL_Arduino = "Booting: LVGL ";
        LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
        Serial.println(LVGL_Arduino);
      }
      void initLVGLDisplay(){
        lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );
        lv_disp_drv_init( &disp_drv );
        disp_drv.hor_res = screenWidth;
        disp_drv.ver_res = screenHeight;
        disp_drv.flush_cb = flush_display;
        disp_drv.draw_buf = &draw_buf;
        lv_disp_drv_register( &disp_drv );
      } 
      void initLVGLInput(){
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init( &indev_drv );
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = read_touchpad;
        lv_indev_drv_register( &indev_drv );
      }
    
    //************ Object Creation **************//
      lv_obj_t * createTab(lv_obj_t * parent){
        lv_obj_t * tab = lv_tabview_add_tab(parent, "");
        lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
        // lv_obj_add_event_cb(tab, reset_sleep_timeout_cb, LV_EVENT_PRESSED, NULL);
        // lv_obj_add_event_cb(tab, reset_sleep_timeout_cb, LV_EVENT_PRESSING, NULL);

        return tab;
      }
    
    //************ Event Handlers **************//
      static void update_AHRS_label(String prefix, lv_obj_t * label, float value){
        String valueStr = String(value, 1);
        valueStr = prefix + valueStr;
        const char * valueArr = valueStr.c_str();
        lv_label_set_text(label, valueArr);
      }
      
      static void update_AHRS_dot(float h, float p, float r){
        float scalar = 1.5;
        p*=scalar;
        r*=scalar;
        
        int x, y;
        x = (int)-r;
        y = (int)-p;

        lv_obj_align(AHRS_dot, LV_ALIGN_CENTER, x, y);
      }
  //~~~~~~~~~~~ Misc Functions ~~~~~~~~~~~~~~~~~~~~~~~//
    void initPSRAM(){
      if(psramInit()){
        Serial.println("\nPSRAM is correctly initialized");
      }else{
        Serial.println("\nPSRAM not available");
      }
    }