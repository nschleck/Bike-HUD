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
    <MadgwickAHRS>-> AHRS algorithm implementation

  Roadmap:
    DONE get display and touch functions running
    TODO implement AHRS
    TODO implement LVGL graphics
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

// Libraries
  #include <TFT_eSPI.h>           // display driver
  #include "CST816S.h"    // touch driver
  #include <lvgl.h>               // graphics library
  #include "lv_conf.h"

// Pins
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

//~~~~~~~~~~ LVGL Declarations ~~~~~~~~~//
  // LV_FONT_DECLARE(netto_black_72);
  // declare display buffers
    static lv_disp_draw_buf_t draw_buf;
    static lv_disp_drv_t disp_drv;
    static lv_color_t buf[ screenWidth * screenHeight / 10 ];

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

  Serial.println("Setup done");
}

void loop() {
  lv_timer_handler();
  delay(5);
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
        if( !touched ){
        // if( 0!=touch.data.points )
            data->state = LV_INDEV_STATE_REL;
        }else{
            data->state = LV_INDEV_STATE_PR;

            /*Set the coordinates*/
            data->point.x = touch.data.x;
            data->point.y = touch.data.y;

            Serial.print("touched at x:"); 
            Serial.print(touch.data.x); 
            Serial.print(", y:"); 
            Serial.println(touch.data.y); 
        }
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
    //************ Event Handlers **************//

  //~~~~~~~~~~~ Misc Functions ~~~~~~~~~~~~~~~~~~~~~~~//
    void initPSRAM(){
      if(psramInit()){
        Serial.println("\nPSRAM is correctly initialized");
      }else{
        Serial.println("\nPSRAM not available");
      }
    }