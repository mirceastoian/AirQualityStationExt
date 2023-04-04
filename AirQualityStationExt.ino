#if !(ESP8266 || ESP32)
  #error This code is intended to run on the ESP8266/ESP32 platform only.
#endif

/* Based on Plantower PMS7003 sensor - 2022 version */
/* Using WeMos D1 mini Pro - clone version known with design and build issues, requiring D3 <-> GND connection at flashing to work */

#include <Wire.h>
#include <SoftwareSerial.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include <Timezone.h>
#include <TimeLib.h>

#define __DEBUG true

#define MYSQL_DEBUG_PORT  Serial
#define _MYSQL_LOGLEVEL_  4 // Debug Level from 0 to 4

#include <MySQL_Generic.h>

#define LED_R_PIN D1
#define LED_G_PIN D2
#define LED_B_PIN D4
#define BUZZER_PIN D5
#define PMS7003_RX_PIN D6
#define PMS7003_TX_PIN D7

const int FRAME_LENGTH = 64; // Specs = 2*13+2
char frameBuffer[FRAME_LENGTH];
int frameLength = FRAME_LENGTH;
bool inDataFrame = false;
int dataOffset = 0;
uint16_t data = 0;
uint16_t checksum = 0;
long int dataReadCount = 0;
const int DATA_READ_COUNT_LIMIT = 43800;

struct pms7003data
{
  uint8_t frameHeader[2];
  uint16_t frameLength = FRAME_LENGTH;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint8_t reserved_high, reserved_low;
  uint16_t checksum;
};

struct pms7003data dataFrame;

SoftwareSerial pmsSerial(PMS7003_RX_PIN, PMS7003_TX_PIN);

const int REFRESH_DELAY_IN_SECONDS = 30;
const int DISPLAY_OFF_HOUR_START = 22; // must be after 12 PM and before 12 AM
const int DISPLAY_OFF_HOUR_END = 6; // must be before 12 PM and after 12 AM
const int ALARM_BEEPS = 2; // How many beeps when threshold is reached

uint16_t pm25Queue[86400 / REFRESH_DELAY_IN_SECONDS];
int queuePosition = 0;
bool isQueueFilled = false;

// PM2.5 CAQI limits
const int PM25_STANDARD_VERY_LOW_THRESHOLD = 15;
const int PM25_STANDARD_LOW_THRESHOLD = 30;
const int PM25_STANDARD_MEDIUM_THRESHOLD = 55;
const int PM25_STANDARD_HIGH_THRESHOLD = 110; // very high is above

// MariaDB
IPAddress mariadb_server(192, 168, 1, 250);
uint16_t server_port           = 3306;
char default_database[]        = "AirQualityStation03";
char default_telemetry_table[] = "telemetry";
char default_log_table[]       = "log";
char db_user[]                 = "***";
char db_password[]             = "***";
MySQL_Connection mdb_conn((Client *)&client);
MySQL_Query *query_mem;

// Logging
String log_buffer = "";
bool SAVE_LOG_TO_DB = true;
const int LOG_BUFFER_LIMIT = 32768;
bool error_while_saving_log_to_db = false;

// WiFi
String device_hostname = "AirQualityStation03";
char wifissid[] = "***";
char wifipass[] = "***";
bool isWifiOn = false;
bool connectOnlyWhenSaving = true;
int wifiConnectionFailedCount = 0;
// Connection retry limit: 1 minute
const int WIFI_DELAY = 2000;
const int WIFI_MAX_RETRY_COUNT = 30;

// Values from https://www.intuitibits.com/2016/03/23/dbm-to-percent-conversion/
int signal_dBM[] = { -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1};
int signal_percent[] = {0, 0, 0, 0, 0, 0, 4, 6, 8, 11, 13, 15, 17, 19, 21, 23, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 46, 48, 50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 69, 71, 72, 73, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 90, 91, 92, 93, 93, 94, 95, 95, 96, 96, 97, 97, 98, 98, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int signalStrength = 0;
int signalStrengthPercentage = 0;

// NTP, date & time
int GTMOffset = 0; // GMT+2 (+3 for summer time)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", GTMOffset*60*60, 60000);
TimeChangeRule EEST = {"EEST", Last, Sun, Mar, 3, 180};    // Eastern European Summer Time
TimeChangeRule EET = {"EET ", Last, Sun, Oct, 4, 120};     // Eastern European (Standard) Time
Timezone EE_TZ(EEST, EET);
// String weekDays[7] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
// String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};


/*
 * Input time in epoch format and return tm time format
 * by Renzo Mischianti <www.mischianti.org> 
 */
tm getDateTimeByParams(long time)
{
    struct tm *newtime;
    const time_t tim = time;
    newtime = localtime(&tim);

    return *newtime;
}

/*
 * Input tm time format and return String with format pattern
 * by Renzo Mischianti <www.mischianti.org>
 */
String getDateTimeStringByParams(tm *newtime, char* pattern = (char *)"%d.%m.%Y %H:%M:%S")
{
  char buffer[30];
  strftime(buffer, 30, pattern, newtime);

  return buffer;
}
 
/*
 * Input time in epoch format format and return String with format pattern
 * by Renzo Mischianti <www.mischianti.org> 
 */
String getEpochStringByParams(long time, char* pattern = (char *)"%d.%m.%Y %H:%M:%S")
{
  tm newtime;
  newtime = getDateTimeByParams(time);

  return getDateTimeStringByParams(&newtime, pattern);
}

void setup()
{
  Serial.begin(9600);

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  WiFi.mode(WIFI_STA);
  // Known not to work, instead reconnect on status change
  // WiFi.setAutoReconnect(true);
  // WiFi.persistent(true);

  connect_to_wifi();
  // Getting time via NTP
  if (isWifiOn)
  {
    timeClient.begin();
    delay(1000);
    if (timeClient.forceUpdate())
    {
      if (__DEBUG) Serial.println(F("Adjusting to local time..."));
      setTime(timeClient.getEpochTime());

      if (__DEBUG)
      {
        Serial.print(F("Current Date & Time: "));
        Serial.println(getEpochStringByParams(EE_TZ.toLocal(now())));
      }
    }
    else
    {
      write_to_log(F("NTP update failed at setup. Retrying later."));
    }

    delay(1000);
  }
  else
  {
    write_to_log(F("Unable connecting to WiFi for NTP sync."));
  }

  delay(1000);
  beep();

  write_to_log(F("Air Quality Station start sequence completed."));
}

bool read_particle_sensor()
{
  pmsSerial.begin(9600);
  bool dataRead = false;

  while (!dataRead)
  {
    if (pmsSerial.available() > 32)
    {
      for (int i = pmsSerial.available(); i > 0; i--) pmsSerial.read();
    }

    if (pmsSerial.available() > 0)
    {
      data = pmsSerial.read();

      if (!inDataFrame)
      {
        if (data == 0x42 && dataOffset == 0)
        {
            frameBuffer[dataOffset] = data;
            dataFrame.frameHeader[0] = data;
            checksum = data;
            dataOffset++;
        }
        else if (data == 0x4D && dataOffset == 1)
        {
            frameBuffer[dataOffset] = data;
            dataFrame.frameHeader[1] = data;
            checksum += data;
            inDataFrame = true;
            dataOffset++;
        }
      }
      else
      {
        frameBuffer[dataOffset] = data;
        checksum += data;
        dataOffset++;

        uint16_t value = frameBuffer[dataOffset - 1] + (frameBuffer[dataOffset - 2] << 8);

        switch (dataOffset)
        {
          case 4:
            dataFrame.frameLength = value;
            frameLength = value + dataOffset;
            break;
          case 6:
            dataFrame.pm10_standard = value;
            break;
          case 8:
            dataFrame.pm25_standard = value;
            break;
          case 10:
            dataFrame.pm100_standard = value;
            break;
          case 12:
            dataFrame.pm10_env = value;
            break;
          case 14:
            dataFrame.pm25_env = value;
            break;
          case 16:
            dataFrame.pm100_env = value;
            break;
          case 18:
            dataFrame.particles_03um = value;
            break;
          case 20:
            dataFrame.particles_05um = value;
            break;
          case 22:
            dataFrame.particles_10um = value;
            break;
          case 24:
            dataFrame.particles_25um = value;
            break;
          case 26:
            dataFrame.particles_50um = value;
            break;
          case 28:
            dataFrame.particles_100um = value;
            break;
          case 29:
            dataFrame.reserved_high = frameBuffer[dataOffset - 1];
            break;
          case 30:
            dataFrame.reserved_low = frameBuffer[dataOffset - 1];
            break;
          case 32:
            dataFrame.checksum = value;
            checksum -= ((value >> 8) + (value & 0xFF));
            break;
          default:
            break;
        }

        if (dataOffset >= frameLength)
        {
          dataReadCount++;
          if (dataReadCount > DATA_READ_COUNT_LIMIT)
          {
            dataReadCount = 1;
            write_to_log(F("Data read count limit was reached and reset."));
          }

          dataRead = true;
          dataOffset = 0;
          inDataFrame = false;
        }
      }
    }
  }

  pmsSerial.end();

  return (checksum == dataFrame.checksum);
}

void display_data()
{
  if (__DEBUG)
  {
    Serial.print(F("Read #"));
    Serial.print(dataReadCount);
    Serial.print(F(" @ "));
    Serial.println(getEpochStringByParams(EE_TZ.toLocal(now())));
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (standard)"));
    Serial.print(F("PM 1.0: ")); Serial.print(dataFrame.pm10_standard);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(dataFrame.pm25_standard);
    Serial.print(F("\t\tPM 10: ")); Serial.println(dataFrame.pm100_standard);
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (environmental)"));
    Serial.print(F("PM 1.0: ")); Serial.print(dataFrame.pm10_env);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(dataFrame.pm25_env);
    Serial.print(F("\t\tPM 10: ")); Serial.println(dataFrame.pm100_env);
    Serial.println(F("---------------------------------------"));
    Serial.print(F("Particles > 0.3um / 0.1L air: ")); Serial.println(dataFrame.particles_03um);
    Serial.print(F("Particles > 0.5um / 0.1L air: ")); Serial.println(dataFrame.particles_05um);
    Serial.print(F("Particles > 1.0um / 0.1L air: ")); Serial.println(dataFrame.particles_10um);
    Serial.print(F("Particles > 2.5um / 0.1L air: ")); Serial.println(dataFrame.particles_25um);
    Serial.print(F("Particles > 5.0um / 0.1L air: ")); Serial.println(dataFrame.particles_50um);
    Serial.print(F("Particles > 10.0um / 0.1L air: ")); Serial.println(dataFrame.particles_100um);
    Serial.println(F("---------------------------------------"));
  }

  if (timeClient.isTimeSet()) // timeClient initializes to 10:00:00 if it does not receive an NTP packet before the 100ms timeout.
  {
    const time_t current = EE_TZ.toLocal(now());
    struct tm *ptm = localtime((time_t *) &current);

    if ((ptm->tm_hour >= DISPLAY_OFF_HOUR_START) || (ptm->tm_hour < DISPLAY_OFF_HOUR_END))
    {
      set_led_off();
    }
    else
    {
      control_led();
    }
  }
  else
  {
    control_led();
  }
}

void control_led()
{
  if (dataFrame.pm25_standard <= PM25_STANDARD_VERY_LOW_THRESHOLD)
  {
    set_led_very_low();
  }
  else if (dataFrame.pm25_standard > PM25_STANDARD_VERY_LOW_THRESHOLD && dataFrame.pm25_standard <= PM25_STANDARD_LOW_THRESHOLD)
  {
    set_led_low();
  }
  else if (dataFrame.pm25_standard > PM25_STANDARD_LOW_THRESHOLD && dataFrame.pm25_standard <= PM25_STANDARD_MEDIUM_THRESHOLD)
  {
    set_led_medium();
  }
  else if (dataFrame.pm25_standard > PM25_STANDARD_MEDIUM_THRESHOLD && dataFrame.pm25_standard <= PM25_STANDARD_HIGH_THRESHOLD)
  {
    set_led_high();
  }
  else if (dataFrame.pm25_standard > PM25_STANDARD_HIGH_THRESHOLD)
  {
    set_led_very_high();
  }
}

int get_pm25_24h_average()
{
  int sum = 0;
  int queueSize = isQueueFilled ? (86400 / REFRESH_DELAY_IN_SECONDS) : queuePosition;

  for (int i = 0; i < queueSize; i++) sum += pm25Queue[i];

  return sum / queueSize;
}

void save_data_to_db()
{
  if (isWifiOn)
  {
    signalStrength = WiFi.RSSI();
    for (int x = 0; x < 100; x = x + 1)
    {
      if (signal_dBM[x] == signalStrength)
      {
        signalStrengthPercentage = signal_percent[x];

        if (__DEBUG)
        {
          Serial.print(F("Signal strength: "));
          Serial.print(signalStrengthPercentage);
          Serial.println("%");
        }
        break;
      }
    }

    // Save values to MariaDB
    if (mdb_conn.connectNonBlocking(mariadb_server, server_port, db_user, db_password) != RESULT_FAIL)
    {
      delay(500);

      String insert_readings_query =
        String(F("INSERT INTO ")) + default_database + "." + default_telemetry_table +
        F(" (pm10_standard, pm25_standard, pm100_standard, pm10_env, pm25_env, pm100_env, ") +
        F("particles_03um, particles_05um, particles_10um, particles_25um, ") +
        F("particles_50um, particles_100um, wifi_signalstrength_p) VALUES (") +
        String(dataFrame.pm10_standard) + ", " +
        String(dataFrame.pm25_standard) + ", " +
        String(dataFrame.pm100_standard) + ", " +
        String(dataFrame.pm10_env) + ", " +
        String(dataFrame.pm25_env) + ", " +
        String(dataFrame.pm100_env) + ", " +
        String(dataFrame.particles_03um) + ", " +
        String(dataFrame.particles_05um) + ", " +
        String(dataFrame.particles_10um) + ", " +
        String(dataFrame.particles_25um) + ", " +
        String(dataFrame.particles_50um) + ", " +
        String(dataFrame.particles_100um) + ", " +
        String(signalStrengthPercentage) +
        ")";

      MySQL_Query query_mem = MySQL_Query(&mdb_conn);

      if (mdb_conn.connected())
      {
        if (!query_mem.execute(insert_readings_query.c_str()))
        {
          write_to_log(F("Query execution failed. Cannot save sensor data.\n") + insert_readings_query);
        }

        if (SAVE_LOG_TO_DB && log_buffer != "")
        {
          MySQL_Query query_mem = MySQL_Query(&mdb_conn);

          if (wifiConnectionFailedCount > 0) log_buffer += F("\nWiFi connection failure count: ") + String(wifiConnectionFailedCount);
          
          String insert_log_query =
            String(F("INSERT INTO ")) + default_database + "." + default_log_table +
            F(" (message) VALUES ('") + log_buffer + "')";

          if (!query_mem.execute(insert_log_query.c_str()))
          {
            write_to_log(F("Saving log buffer failed."));
            error_while_saving_log_to_db = true;
          }
          else
          {
            log_buffer = "";
            error_while_saving_log_to_db = false;
          }
        }
      }
      else
      {
        write_to_log(F("Disconnected from MariaDB server. Cannot save sensor data."));
      }

      delay(500);
      mdb_conn.close();
    } 
    else 
    {
      write_to_log("Unable connecting to MariaDB.");
    }

    delay(500);
  }
  else
  {
    write_to_log("No active WiFi connection to save sensor data. Retrying.");
  }
}

void loop()
{
  isWifiOn = (WiFi.status() == WL_CONNECTED);
  if (!isWifiOn) connect_to_wifi();

  if (!read_particle_sensor())
  {
    write_to_log(F("Invalid checksum."));
  }
  else
  {
    pm25Queue[queuePosition] = dataFrame.pm25_standard;
    queuePosition++;

    if (queuePosition == 86400 / REFRESH_DELAY_IN_SECONDS)
    {
      queuePosition = 0;
      isQueueFilled = true;
    }

    if (timeClient.forceUpdate())
    {
      delay(1000);
      setTime(timeClient.getEpochTime());
    }
    else
    {
      if (__DEBUG) Serial.println(F("NTP update failed at runtime. Retrying later."));
    }

    display_data();
    save_data_to_db();
  }

  delay(REFRESH_DELAY_IN_SECONDS * 1000);
}

void beep()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

void connect_to_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.hostname(device_hostname.c_str());
  WiFi.begin(wifissid, wifipass);
  if (__DEBUG) Serial.print("Connecting to WiFi...");

  int wifiRetryCount = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetryCount < WIFI_MAX_RETRY_COUNT)
  {
    delay(WIFI_DELAY);
    Serial.print(".");

    wifiRetryCount++;
  }

  isWifiOn = (WiFi.status() == WL_CONNECTED);

  if (isWifiOn)
  {
    if (__DEBUG)
    {
      Serial.print(F("\nConnected to: "));
      Serial.println(wifissid);
      Serial.print(F("IP address: "));
      Serial.println(WiFi.localIP());
    }

    timeClient.update();

    wifiConnectionFailedCount = 0;
  }
  else
  {
    WiFi.disconnect(); // Stop trying
    wifiConnectionFailedCount++;

    if (__DEBUG) Serial.println();
    write_to_log(F("Unable connecting to WiFi."));
  }
}

void write_to_log(String message)
{
  // When log buffer exceeds the set limit and cannot be saved in the database stop logging
  if ((log_buffer.length() + message.length() <= LOG_BUFFER_LIMIT) && !error_while_saving_log_to_db)
  {
    log_buffer += String(log_buffer != "" ? "\n" : "") + "[" + getEpochStringByParams(EE_TZ.toLocal(now())) + "] " + message;
  }

  if (__DEBUG) Serial.println(message);
}

void set_led_very_low()
{
  analogWrite(LED_R_PIN, 121);
  analogWrite(LED_G_PIN, 188);
  analogWrite(LED_B_PIN, 106);
}

void set_led_low()
{
  analogWrite(LED_R_PIN, 187);
  analogWrite(LED_G_PIN, 207);
  analogWrite(LED_B_PIN, 76);
}

void set_led_medium()
{
  analogWrite(LED_R_PIN, 238);
  analogWrite(LED_G_PIN, 194);
  analogWrite(LED_B_PIN, 11);
}

void set_led_high()
{
  analogWrite(LED_R_PIN, 242);
  analogWrite(LED_G_PIN, 147);
  analogWrite(LED_B_PIN, 5);
}

void set_led_very_high()
{
  analogWrite(LED_R_PIN, 232);
  analogWrite(LED_G_PIN, 65);
  analogWrite(LED_B_PIN, 111);
}

void set_led_off()
{
  analogWrite(LED_R_PIN, 0);
  analogWrite(LED_G_PIN, 0);
  analogWrite(LED_B_PIN, 0);
}

/* TODO:
  1.
*/