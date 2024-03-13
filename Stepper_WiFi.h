// ------------------------------------------------------------------
// WiFi Server

#include <WiFi101.h>

#define FREQ_CHANGE 0
#define AMP_CHANGE 1
#define WAIT_AT_EXTREMES 2

#define AMP_CHANGE_VALUE -999
#define WAIT_AT_EXTREMES_VALUE -888

// By default an access point with ssid_AP name is created, but the device can
// also be connected to an existing network with ssid and pass. To enable this
// modify setup function in ino file.

char ssid_AP[] = "MKR1000";

char ssid[] = "";        // your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)

int bufferIdx = 0;
char buffer[10];  // array to store incoming chars
char TERMINATOR_CHAR = '\n';  // message end char

int CHANGE_OK = 1;  // OK value sent when frequency has been changed
float AMPLITUDE_FROM_WIFI;  // Amplitude value received via WiFi
float NEW_FREQ_FROM_WIFI;  // Freq. value sent via WiFi

int status = WL_IDLE_STATUS;

WiFiServer server(23);
WiFiClient client;

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setupWiFiAP() {
  Serial.println("[MKR1000 AP] Access Point Web Server");

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // by default the local IP address of will be 192.168.1.1
  // print the network name (SSID)
  Serial.print("[MKR1000 AP] Creating access point named: ");
  Serial.println(ssid_AP);

  // Create open network. Change this line if you want to create a WEP network:
  status = WiFi.beginAP(ssid_AP);
  if (status != WL_AP_LISTENING) {
    Serial.println("[MKR1000 AP] Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 5 seconds for connection:
  delay(5000);

  // start the web server
  server.begin();

  Serial.println("[MKR1000 AP] Access point created!");

  // you're connected now, so print out the status
  printWiFiStatus();
}

void connectToWiFi() {
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // start the server:
  server.begin();
  // you're connected now, so print out the status
  printWiFiStatus();
}

bool waitForClient() {
  client.stop(); // Reset any previous connection
  Serial.println("[MKR1000 CLIENT] Waiting for client...");

  while (!client) {
    client = server.available();
  }

  // clear out the input buffer:
  client.flush();
  Serial.println("[MKR1000 CLIENT] Client connected!");

  return client;
}

bool isClientConnected() {
  return client.connected();
}

float readFromWiFiClient() {
  float value = -1;

  // wait for a new client (this is moved to setup to only connect once)
  //client = server.available();

  // when the client sends the first byte
  if (client) {
    if (client.available() > 0) {
      // read the bytes incoming from the client:
      char thisChar = client.read();
      // echo the bytes to the server as well:
      //Serial.write(thisChar);

      if (thisChar != TERMINATOR_CHAR) {
        buffer[bufferIdx++] = thisChar;
      }
      else {
        buffer[bufferIdx] = '\0'; // terminating character
        if (buffer[bufferIdx - 1] == 'A'){
          // New amplitude change
          value = AMP_CHANGE_VALUE;
        } else if (buffer[bufferIdx - 1] == 'X') {
          // Wait at extremes mode
          value = WAIT_AT_EXTREMES_VALUE;
        } else {
          // New freq. value
          value = atof(buffer);
        }
        bufferIdx = 0;
      }
    }
  }

  return value;
}

bool amplitudeRequestedViaWiFi() {
  AMPLITUDE_FROM_WIFI = readFromWiFiClient();
  if (AMPLITUDE_FROM_WIFI == -1) return false;
  else{
    Serial.print("[MKR1000 CLIENT] Amplitude = ");
    Serial.print(AMPLITUDE_FROM_WIFI, 4);
    Serial.println(" mm");
    return true;
  }
}

int newRequestViaWiFi() {
  float read = readFromWiFiClient();

  // Nothing requested
  if (read == -1) return -1;

  // Amplitude change requested
  if (read == AMP_CHANGE_VALUE){
    Serial.println("[MKR1000 CLIENT] Amplitude change...");
    while (!amplitudeRequestedViaWiFi());
    return AMP_CHANGE;
  }
  // Wait at extreme requested
  else if (read == WAIT_AT_EXTREMES_VALUE){
    Serial.println("[MKR1000 CLIENT] Wait at oscillation extremes...");
    return WAIT_AT_EXTREMES;
  }

  // Frequency change requested
  else {
    NEW_FREQ_FROM_WIFI = read;
    Serial.print("[MKR1000 CLIENT] Freq. = ");
    Serial.print(NEW_FREQ_FROM_WIFI, 4);
    Serial.println(" Hz");
    return FREQ_CHANGE;
  }
}

void send_CHANGE_OK_WiFi(){
  Serial.println("[MKR1000 CLIENT] Sending OK...");
  client.write(CHANGE_OK);
}