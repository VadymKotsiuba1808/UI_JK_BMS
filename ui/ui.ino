#include <lvgl.h>
#include "FS.h"
#include <TFT_eSPI.h>
#include <TFT_Touch.h>
#include <ui.h>

/* Libraries for WiFi OTA */
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

/* Define macro for enable or disable debug software serial */
/* Uncomment for enabling or comment for disabling */
#define DEBUG_SERIAL
#define DbgSerial Serial

AsyncWebServer server(80);
#define AP_SSID "ESP32 JKBMS UI"

/*Don't forget to set Sketchbook location in File/Preferencesto the path of your UI project (the parent foder of this INO file)*/

uint32_t lcd_timer = 0;
uint32_t bms_update_timer = 0;

bool isUpdateOtaNow = false;

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenHeight, screenWidth); /* TFT instance */

// These are the pins used to interface between the 2046 touch controller and Arduino Pro
#define DOUT 39  /* Data out pin (T_DO) of touch screen */
#define DIN  32  /* Data in pin (T_DIN) of touch screen */
#define DCS  33  /* Chip select pin (T_CS) of touch screen */
#define DCLK 25  /* Clock pin (T_CLK) of touch screen */

/* Create an instance of the touch screen library */
TFT_Touch touch = TFT_Touch(DCS, DCLK, DIN, DOUT);

#define SerialBms Serial1
#define BMS_BAUDRATE 115200
#define BMS_TX 22
#define BMS_RX 35

/* Error codes defines */
#define COMMON_ERROR -1
#define NOT_VALID_RESP (COMMON_ERROR)
#define NO_MEM_FOR_RESP (COMMON_ERROR)
#define NO_INDEX_IN_ARRAY (COMMON_ERROR)

#if LV_USE_LOG != 0 && defined(DEBUG_SERIAL)
  /* Serial debugging */
  void my_print(const char * buf)
  {
      DbgSerial.printf(buf);
  }
#endif

void setModemSleep(void)
{
  WiFi.mode(WIFI_OFF);    // Switch WiFi off
  setCpuFrequencyMhz(160);
}

void wakeModemSleep(void)
{
  setCpuFrequencyMhz(240);
}

/* Callback for screen loaded event(When Screen2 loaded we need to start OTA update) */
void event_screen2_loaded(lv_event_t * e)
{
  wakeModemSleep();
  WiFi.mode(WIFI_AP);
  WiFi.enableAP(true);
  WiFi.softAP(AP_SSID);
  AsyncElegantOTA.begin(&server);
  server.begin();
  digitalWrite(TFT_BL, HIGH);

  isUpdateOtaNow = true;
}

/* Back button on Screen2 was clicked and we need to back on main screen */
void event_back_btn_clicked(lv_event_t * e)
{
  server.end();
  WiFi.enableAP(false);
  setModemSleep();

  isUpdateOtaNow = false;
}

void lcd_timer_check(uint32_t delay_to_sleep)
{
  if((millis() >= lcd_timer+delay_to_sleep) && (!isUpdateOtaNow))
  {
    tft.writecommand(0x10); // LCD go to sleep
    #if defined(DEBUG_SERIAL)
      DbgSerial.println("LCD BL OFF");
    #endif
    digitalWrite(TFT_BL, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, LOW);
    delay(100);
    esp_deep_sleep_start();
  }
}

void lcd_timer_reset(void)
{
  lcd_timer = millis();
}

int16_t arr_find_index(uint8_t arr[], uint16_t size, uint8_t target)
{
  for(uint8_t i=0; i<size; i++)
  {
    if (arr[i]==target) return i;
  }
  return NO_INDEX_IN_ARRAY;
}

void serial_bms_flush(){
  while(SerialBms.available() > 0) {
    char t = SerialBms.read();
  }
}

int8_t read_bms_data(uint32_t delay_read)
{
  if((millis() >= bms_update_timer+delay_read) && (!isUpdateOtaNow))
  {
    #if defined(DEBUG_SERIAL)
      DbgSerial.println("read_bms_data");
      DbgSerial.print("bms_update_timer:");
      DbgSerial.println(bms_update_timer);
      DbgSerial.print("delay_read:");
      DbgSerial.println(delay_read);
      DbgSerial.print("bms_update_timer+delay_read:");
      DbgSerial.println(bms_update_timer+delay_read);
      DbgSerial.print("millis():");
      DbgSerial.println(millis());
    #endif
    uint8_t arr[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};
    uint8_t arr_size = sizeof(arr)/sizeof(arr[0]);

    /* 
     * Workaround for BMS read we need to send three packets 
     * After first no response, after second BMS respond unexpected data
     * and after third we got actual data.
     *
     * We need to do three packet send only one time after power init
     * All next read_bms_data could be done with one packet.
     *
     */
    static bool first_entry = true;
    if(first_entry)
    {
      first_entry = false;
      SerialBms.write(arr, arr_size);
      delay(100);
      serial_bms_flush();
      delay(100);
      SerialBms.write(arr, arr_size);
      delay(100);
      serial_bms_flush();
      delay(100);
    }

    SerialBms.write(arr, arr_size);
    
    #if defined(DEBUG_SERIAL)
      DbgSerial.print("Send request packet to BMS:");
      for (int i = 0; i < arr_size; i++) 
      {
        DbgSerial.print(arr[i], HEX);
        DbgSerial.print(" ");
      }
      DbgSerial.println("");
    #endif

    bool isReading = true;

    uint8_t state = 0;
    uint8_t rxByte;
    uint8_t *response = NULL;
    uint32_t response_size = 0;
    uint32_t index = 0;


    while(isReading)
    {
      lv_timer_handler();
      if(SerialBms.available())
      {
        rxByte = SerialBms.read();
        response_size++;
        if(response_size == 1)
        {
          response = (uint8_t*)calloc(response_size, sizeof(uint8_t));
        }
        else
        {
          response = (uint8_t*)realloc(response, response_size);
        }
        if (!response) {
          return NO_MEM_FOR_RESP;
        }

        response[index++] = rxByte;

        // End packet combination
        // 0x00, 0x00, 0x00, 0x00, 
        switch(state)
        {
          case 0:
          case 1:
          case 2:
          case 3: 
            rxByte == 0x00 ? state++ : state=0;
            break;

          case 4: 
            if(rxByte == 0x68)
            {
              // Add memory for last checksum 4 bytes
              response_size += 4;
              response = (uint8_t*)realloc(response, response_size);
              if (!response) {
                return NO_MEM_FOR_RESP;
              }
              while(SerialBms.available()<4) lv_timer_handler();
              for (uint8_t i=0; i<4; i++)
              {
                rxByte = SerialBms.read();
                response[index++] = rxByte;
              }
              isReading = false;
            }
            else state = 0;
            break;

          default:
            state = 0;
            break;
        }
      }
    }

    #if defined(DEBUG_SERIAL)
      DbgSerial.print("Size of received data from BMS: ");
      DbgSerial.println(response_size);
   

      DbgSerial.println("Received data from BMS: ");
      for (int i = 0; i < response_size; i++) 
      {
        DbgSerial.print(response[i], HEX);
        DbgSerial.print(" ");
      }
    #endif

    /*
     * If response is not valid please check start combination
     * or similar dependency
     * Or checksum in the end
     */
    if((response[0] != 0x4E) || (response[1] != 0x57))
    {
      return NOT_VALID_RESP;
    }

    uint32_t checksum = 0;
    // Check checksum
    for (int i = 0; i < response_size-2; i++) 
    {
      checksum+=response[i];
    }
    uint32_t checksum_in_resp = (response[response_size-2]<<8) | (response[response_size-1]);
    if(checksum != checksum_in_resp) return NOT_VALID_RESP;

    int16_t found_index = 0;
    int32_t temp_int = 0;
    float temp_float = 0.0f;
    char temp_str[50];

    /******************************************************* 
     *********** Find setted parameters from BMS ***********
     *******************************************************/
    
    found_index = arr_find_index(response, response_size, 0x8E);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index++;
    float setted_total_overvolt = ((response[found_index] << 8) | response[found_index+1])/100.0; // setted Total Overvoltage

    found_index = arr_find_index(response, response_size, 0x8F);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index++;
    float setted_total_undervolt = ((response[found_index] << 8) | response[found_index+1])/100.0; // setted Total Undervoltage

    found_index = arr_find_index(response, response_size, 0xAA);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index+=3;
    float setted_battery = (float) ((response[found_index] << 8) | response[found_index+1]); // setted Battery Cap

    /******************************************************* 
     *********** Find values and update data on lcd ********
     *******************************************************/
    // Find cells voltage
    lv_obj_t* minCellLabel = ui_CellLabel1;
    lv_obj_t* maxCellLabel = ui_CellLabel1;
    float min_CellVolt = 0.0f;
    float max_CellVolt = 0.0f;
    float avrg_cell_colt = 0.0f;

    found_index = arr_find_index(response, response_size, 0x79);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index+=3;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/1000.0; // Cell 1
    avrg_cell_colt += temp_float;
    min_CellVolt = temp_float;
    max_CellVolt = temp_float;
    sprintf(temp_str, "%.3fV", temp_float);
    lv_label_set_text(ui_CellLabel1, temp_str);
    found_index+=3;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/1000.0; // Cell 2
    avrg_cell_colt += temp_float;
    if(temp_float < min_CellVolt) 
    {
      min_CellVolt = temp_float;
      minCellLabel = ui_CellLabel2;
    }
    if(temp_float > max_CellVolt) 
    {
      max_CellVolt = temp_float;
      maxCellLabel = ui_CellLabel2;
    }
    sprintf(temp_str, "%.3fV", temp_float);
    lv_label_set_text(ui_CellLabel2, temp_str);
    found_index+=3;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/1000.0; // Cell 3
    avrg_cell_colt += temp_float;
    if(temp_float < min_CellVolt) 
    {
      min_CellVolt = temp_float;
      minCellLabel = ui_CellLabel3;
    }
    if(temp_float > max_CellVolt) 
    {
      max_CellVolt = temp_float;
      maxCellLabel = ui_CellLabel3;
    }
    sprintf(temp_str, "%.3fV", temp_float);
    lv_label_set_text(ui_CellLabel3, temp_str);
    found_index+=3;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/1000.0; // Cell 4
    avrg_cell_colt += temp_float;
    if(temp_float < min_CellVolt) 
    {
      min_CellVolt = temp_float;
      minCellLabel = ui_CellLabel4;
    }
    if(temp_float > max_CellVolt) 
    {
      max_CellVolt = temp_float;
      maxCellLabel = ui_CellLabel4;
    }
    sprintf(temp_str, "%.3fV", temp_float);
    lv_label_set_text(ui_CellLabel4, temp_str);
    found_index+=3;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/1000.0; // Cell 5
    avrg_cell_colt += temp_float;
    if(temp_float < min_CellVolt) 
    {
      min_CellVolt = temp_float;
      minCellLabel = ui_CellLabel5;
    }
    if(temp_float > max_CellVolt) 
    {
      max_CellVolt = temp_float;
      maxCellLabel = ui_CellLabel5;
    }
    sprintf(temp_str, "%.3fV", temp_float);
    lv_label_set_text(ui_CellLabel5, temp_str);
    found_index+=3;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/1000.0; // Cell 6
    avrg_cell_colt += temp_float;
    if(temp_float < min_CellVolt) 
    {
      min_CellVolt = temp_float;
      minCellLabel = ui_CellLabel6;
    }
    if(temp_float > max_CellVolt) 
    {
      max_CellVolt = temp_float;
      maxCellLabel = ui_CellLabel6;
    }
    sprintf(temp_str, "%.3fV", temp_float);
    lv_label_set_text(ui_CellLabel6, temp_str);
    found_index+=3;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/1000.0; // Cell 7
    avrg_cell_colt += temp_float;
    if(temp_float < min_CellVolt) 
    {
      min_CellVolt = temp_float;
      minCellLabel = ui_CellLabel7;
    }
    if(temp_float > max_CellVolt) 
    {
      max_CellVolt = temp_float;
      maxCellLabel = ui_CellLabel7;
    }
    sprintf(temp_str, "%.3fV", temp_float);
    lv_label_set_text(ui_CellLabel7, temp_str);

    avrg_cell_colt /= 7.0;
    sprintf(temp_str, "%.3fV", avrg_cell_colt);
    lv_label_set_text(ui_AvrgVolLabel, temp_str);

    sprintf(temp_str, "%.3fV", (max_CellVolt-min_CellVolt));
    lv_label_set_text(ui_DeltaVolLabel, temp_str);

    lv_obj_clear_state(ui_CellLabel1, LV_STATE_USER_1|LV_STATE_USER_2);
    lv_obj_clear_state(ui_CellLabel2, LV_STATE_USER_1|LV_STATE_USER_2);
    lv_obj_clear_state(ui_CellLabel3, LV_STATE_USER_1|LV_STATE_USER_2);
    lv_obj_clear_state(ui_CellLabel4, LV_STATE_USER_1|LV_STATE_USER_2);
    lv_obj_clear_state(ui_CellLabel5, LV_STATE_USER_1|LV_STATE_USER_2);
    lv_obj_clear_state(ui_CellLabel6, LV_STATE_USER_1|LV_STATE_USER_2);
    lv_obj_clear_state(ui_CellLabel7, LV_STATE_USER_1|LV_STATE_USER_2);

    lv_obj_add_state(minCellLabel, LV_STATE_USER_2);
    lv_obj_add_state(maxCellLabel, LV_STATE_USER_1);

    // Find MOS Temp
    found_index = arr_find_index(response, response_size, 0x80);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index+=2;
    temp_int = response[found_index]>100 ? 100-response[found_index] : response[found_index]; // MOS Temp
    sprintf(temp_str, "%dC", temp_int);
    lv_label_set_text(ui_TMOSLabel, temp_str);

    // Find BMS Temp
    found_index = arr_find_index(response, response_size, 0x81);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index+=2;
    temp_int = response[found_index]>100 ? 100-response[found_index] : response[found_index]; // BMS Temp
    sprintf(temp_str, "%dC", temp_int);
    lv_label_set_text(ui_TBMSLabel, temp_str);

    // Find BAT Temp
    found_index = arr_find_index(response, response_size, 0x82);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index+=2;
    temp_int = response[found_index]>100 ? 100-response[found_index] : response[found_index]; // BAT Temp
    sprintf(temp_str, "%dC", temp_int);
    lv_label_set_text(ui_TBATLabel, temp_str);
    
    found_index = arr_find_index(response, response_size, 0x83);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index++;
    temp_float = ((response[found_index] << 8) | response[found_index+1])/100.0; // Total Volt
    sprintf(temp_str, "%.2fV", temp_float);
    lv_label_set_text(ui_BatVoltLabel, temp_str);
    lv_obj_clear_state(ui_BatVoltLabel, LV_STATE_USER_1|LV_STATE_USER_2);
    if(temp_float>=setted_total_overvolt) lv_obj_add_state(ui_BatVoltLabel, LV_STATE_USER_1);
    if(temp_float<=setted_total_undervolt) lv_obj_add_state(ui_BatVoltLabel, LV_STATE_USER_2);

    found_index = arr_find_index(response, response_size, 0x84);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index++;
    temp_int = ((response[found_index] << 8) | response[found_index+1]); // Current Data
    if(temp_int & (1<<15))
    {
      temp_int &= ~(1<<15);
    }
    else
    {
      temp_int &= ~(1<<15); 
      temp_int*=(-1);
    }
    sprintf(temp_str, "%.1fA", temp_int/100.0);
    lv_label_set_text(ui_CurVoltLabel, temp_str);

    found_index = arr_find_index(response, response_size, 0x85);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index++;
    temp_int = response[found_index];
    sprintf(temp_str, "%d%%", temp_int);
    lv_label_set_text(ui_BatteryLabel, temp_str);
    lv_arc_set_value(ui_BatteryArc, temp_int);
    lv_obj_clear_state(ui_BatteryArc, LV_STATE_USER_1|LV_STATE_USER_2);
    if(temp_int>=20 && temp_int<=35) lv_obj_add_state(ui_BatteryArc, LV_STATE_USER_2);
    if(temp_int<20) lv_obj_add_state(ui_BatteryArc, LV_STATE_USER_1);

    // Print remaining capacity
    sprintf(temp_str, "%.1fAh", (setted_battery*(temp_int/100.0)));
    lv_label_set_text(ui_RemCapLabel, temp_str);

    found_index = arr_find_index(response, response_size, 0x8B);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index++;
    temp_int = (response[found_index] << 8) | response[found_index+1]; // Alarm flags
    if (temp_int!=0) 
    {
      lv_label_set_text(ui_AlarmLabel, "Alarm");
      lv_obj_add_state(ui_AlarmLabel, LV_STATE_USER_1);
    }
    else
    {
      lv_label_set_text(ui_AlarmLabel, "Normal");
      lv_obj_clear_state(ui_AlarmLabel, LV_STATE_USER_1);
    }

    !(temp_int&(1<<0)) ? lv_obj_add_flag(ui_AlarmLowCapacity, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmLowCapacity, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<1)) ? lv_obj_add_flag(ui_AlarmMosOverTemp, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmMosOverTemp, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<2)) ? lv_obj_add_flag(ui_AlarmChargeOverVolt, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmChargeOverVolt, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<3)) ? lv_obj_add_flag(ui_AlarmDischargeUnderVolt, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmDischargeUnderVolt, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<4)) ? lv_obj_add_flag(ui_AlarmBatOverTemp, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmBatOverTemp, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<5)) ? lv_obj_add_flag(ui_AlarmChargingOverCur, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmChargingOverCur, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<6)) ? lv_obj_add_flag(ui_AlarmDischargeOverCur, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmDischargeOverCur, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<7)) ? lv_obj_add_flag(ui_AlarmCellDifferential, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmCellDifferential, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<8)) ? lv_obj_add_flag(ui_AlarmSensor2OverTemp, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmSensor2OverTemp, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<9)) ? lv_obj_add_flag(ui_AlarmBatLowTemp, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmBatLowTemp, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<10)) ? lv_obj_add_flag(ui_AlarmCellOverVolt, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmCellOverVolt, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<11)) ? lv_obj_add_flag(ui_AlarmCellUnderVolt, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_AlarmCellUnderVolt, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<12)) ? lv_obj_add_flag(ui_Alarm309AProtection, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_Alarm309AProtection, LV_OBJ_FLAG_HIDDEN);
    !(temp_int&(1<<13)) ? lv_obj_add_flag(ui_Alarm309BProtection2, LV_OBJ_FLAG_HIDDEN) : lv_obj_clear_flag(ui_Alarm309BProtection2, LV_OBJ_FLAG_HIDDEN);

    found_index = arr_find_index(response, response_size, 0x8C);
    if(found_index == NO_INDEX_IN_ARRAY) return NOT_VALID_RESP;
    found_index++;
    temp_int = (response[found_index] << 8) | response[found_index+1]; // Alarm flags
    (temp_int&(1<<0)) ? lv_label_set_text(ui_ChargeLabel, "ON") : lv_label_set_text(ui_ChargeLabel, "OFF");
    (temp_int&(1<<1)) ? lv_label_set_text(ui_DischargeLabel, "ON") : lv_label_set_text(ui_DischargeLabel, "OFF");
    (temp_int&(1<<2)) ? lv_label_set_text(ui_BalanceLabel, "ON") : lv_label_set_text(ui_BalanceLabel, "OFF");

    bms_update_timer = millis();
    free(response);
    /* Workaround. For first run or after deepsleep enable backlight only after data update*/
    digitalWrite(TFT_BL, HIGH);
  }
  return 1;
}

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX, touchY;

    bool touched = touch.Pressed();//tft.getTouch( &touchX, &touchY, 600 );

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {   
        lcd_timer_reset();
        touchX = touch.X();
        touchY = touch.Y();
        
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
 
        #if defined(DEBUG_SERIAL)
          DbgSerial.print( "Data x " );
          DbgSerial.println( touchX );

          DbgSerial.print( "Data y " );
          DbgSerial.println( touchY );
        #endif
    }
}

void setup()
{
    #if defined(DEBUG_SERIAL)
      DbgSerial.begin(115200);
    #endif

    SerialBms.begin(BMS_BAUDRATE, SERIAL_8N1, BMS_RX, BMS_TX);

    #if defined(DEBUG_SERIAL)
      String LVGL_Version = "LVGL ";
      LVGL_Version += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

      DbgSerial.println( LVGL_Version );
    #endif

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    digitalWrite(TFT_BL, LOW);
    tft.setRotation( 3 ); /* Landscape orientation, flipped */
    touch.setRotation(3);

    // Calibrate the touch screen and retrieve the scaling factors
    
    // 1-st Board cal values(only microUSB)
    touch.setCal(468, 3366, 685, 3518, 320, 240, 1);
    
    // 2-nd Board cal values(with Type-C)
    //touch.setCal(501, 3477, 629, 3523, 320, 240, 1);

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();
    #if defined(DEBUG_SERIAL)
      DbgSerial.println( "Setup done" );
    #endif

    uint8_t Freq = getXtalFrequencyMhz();
    #if defined(DEBUG_SERIAL)
      DbgSerial.print("XTAL Freq = ");
      DbgSerial.print(Freq);
      DbgSerial.println(" MHz");
    #endif

    setModemSleep();

    lcd_timer = millis();
    bms_update_timer = millis();
}

void loop()
{
    read_bms_data(2000);
    lcd_timer_check(1000*60*10);
    lv_timer_handler(); /* let the GUI do its work */
    delay(5);
}