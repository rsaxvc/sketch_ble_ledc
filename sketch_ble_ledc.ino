#include <bluefruit.h>
#include <Arduino.h>

#define LED_MAX 255
#define NUM_LED 2

/* LED Controller Service Definitions */
static const BLEUuid LEDS_UUID((const uint8_t*)"RSAXVC|UUID|SLED" );//UUID for LED Service
static const BLEUuid LEDI_UUID((const uint8_t*)"RSAXVC|UUID|ILED" );//UUID for LED Info
static const BLEUuid LEDC_UUID((const uint8_t*)"RSAXVC|UUID|CLED" );//UUID for LED Control
static BLEService        leds = BLEService(LEDS_UUID);
static BLECharacteristic ledi = BLECharacteristic(LEDI_UUID);
static BLECharacteristic ledc = BLECharacteristic(LEDC_UUID);

static BLEDis bledis;    // DIS (Device Information Service) helper class instance
static BLEBas blebas;    // BAS (Battery Service) helper class instance

static uint8_t leddata[NUM_LED];

// Advanced function prototypes
static void setupAdv(void);
static void setupLED(void);
static void connect_callback(uint16_t conn_handle);
static void disconnect_callback(uint16_t conn_handle, uint8_t reason);

void setup()
{
  Serial.begin(115200);
  Serial.println("RSAXVC.NET LED Controller");

  //Need to blank LEDs to kick on underlying HWM hardware
  analogWrite( LED_RED, 0 );
  analogWrite( LED_BLUE, 0 );

  //This is better for the MOSFETs.
  //16MHz is...not so good for MOSFETs.
  //Plus it makes a sweet flicker at high speed!
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_128); // freq = 128kHz
  HwPWM1.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_128); // freq = 128kHz
  HwPWM2.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_128); // freq = 128kHz

  // Initialise the Bluefruit module
  Serial.println("Initializing Bluefruit");
  Bluefruit.begin();

  /* Setup the connection interval
   * Set the min as low as    it can go -  7.5mS = 12 * 0.625mS
   * Set the max as low as Apple can go - 15.0mS = 24 * 0.625mS
   * For sweet FX, we need maximum throughputs.
   * 
   * Units of 0.625ms per LSB
   */
  Bluefruit.setConnInterval(12, 24);

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name");
  Bluefruit.setName("LEDC1");

  // Set the connect/disconnect callback handlers
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("RSAXVC.NET");
  bledis.setModel("LEDC1");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the LED service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the LED Service");
  setupLED();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  setupAdv();

  // Start Advertising
  Serial.println("\nAdvertising");
  Bluefruit.Advertising.start();
}

void setupAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include LED Service UUID
  Bluefruit.Advertising.addService(leds);

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
}

/**
 * Callback when received new LED data
 * @param chr
 * @param data
 * @param len
 * @param offset
 */
void ledc_rxd_cb(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset)
{
  switch( len )
  {
    default:
    case 2:
      leddata[1] = data[1];
      analogWrite( LED_RED, data[1] );
    case 1:
      leddata[0] = data[1];
      analogWrite( LED_BLUE, data[0] );
    case 0:
      break;
  }
  ledc.write( (const void*)leddata, sizeof(leddata) );
}



void setupLED(void)
{
  // Configure the RSAXVC LED service
  // See: 
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // LED INFO                     0x????  Mandatory   Read
  // LED CONTROL                  0x????  Mandatory   Read,WriteNoResp
  leds.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the LED Info characteristic
  // Permission = Read
  // Min Len    = 1
  // Max Len    = 1
  //This tells you how many LEDs there are. That is all.
  ledi.setProperties(CHR_PROPS_READ);
  ledi.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  ledi.setFixedLen(1);
  ledi.begin();
  static const uint8_t leddata = NUM_LED;
  ledi.write(&leddata, sizeof(leddata));

  // Configure the LED Control characteristic
  // See: 
  // Permission = Read, Write No Response
  // Min Len    = 1
  // Max Len    = No limit
  // Each byte corresponds to the level set for an LED.
  // You can read it to see what they're set to.
  // But it's more likely you'll want to write very quickly to change them over time.
  ledc.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE_WO_RESP);
  ledc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  ledc.setWriteCallback(ledc_rxd_cb);
  ledc.write( (const void*)leddata, sizeof(leddata) );
  
  ledc.setFixedLen(NUM_LED);
  ledc.begin();
  ledc.write(0);
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  analogWrite( LED_RED, 0 );
  analogWrite( LED_BLUE, 0 );
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
  memset(leddata,0x00,sizeof(leddata));
  analogWrite( LED_RED, 0 );
  analogWrite( LED_BLUE, 0 );
  Serial.println("Advertising!");
}

void loop()
{
  digitalToggle(LED_RED);
  
  if ( Bluefruit.connected() ) {
    // Just wait for updates to come over BLE
    delay(1000);
  }
  else     // Apply some UI Juice
  {
   static uint32_t val = 0;
   static int8_t dir = 1;
   analogWrite( LED_BLUE, val );
   analogWrite( LED_RED, LED_MAX - val );
   delay(1);
   val+=dir;
   if( val == LED_MAX ) dir = -1;
   else if( val == 0 ) dir = 1;
  }
}

/**
 * RTOS Idle callback is automatically invoked by FreeRTOS
 * when there are no active threads. E.g when loop() calls delay() and
 * there is no bluetooth or hw event. This is the ideal place to handle
 * background data.
 * 
 * NOTE: FreeRTOS is configured as tickless idle mode. After this callback
 * is executed, if there is time, freeRTOS kernel will go into low power mode.
 * Therefore waitForEvent() should not be called in this callback.
 * http://www.freertos.org/low-power-tickless-rtos.html
 * 
 * WARNING: This function MUST NOT call any blocking FreeRTOS API 
 * such as delay(), xSemaphoreTake() etc ... for more information
 * http://www.freertos.org/a00016.html
 */
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
    __asm volatile( "wfi" );
}
