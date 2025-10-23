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
    TODO get display and touch functions running
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







void setup() {

}

void loop() {

}
