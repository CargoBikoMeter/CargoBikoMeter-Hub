/*
 * Copyright (C) 2021 Cargobikometer Contributors
 * Contact: SAI-Lab Berlin (https://www.chemie.tu-berlin.de/sai_lab/sei_real_labor_sai_lab)
 * 
 * This file is part of the Cargobikometer firmware.
 *
 * The Cargobikometer firmware is free software: you can
 * redistribute it and/or modify it under the terms of the GNU
 * Lesser General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cargobikometer firmware is distributed in the hope that
 * it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with the Cargobikometer firmware. If not,
 * see <http://www.gnu.org/licenses/>.
 */

// Based on https://github.com/openbikesensor/OpenBikeSensorFirmware/
// The OpenBikeSensor firmware is free software: you can
// redistribute it and/or modify it under the terms of the GNU
// Lesser General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option)
// any later version.


#include "globals.h"

// Include certificate data for local https connection
#include "https/cert.h"
#include "https/private_key.h"

extern const char * CBMVersion;
extern void showConfigServerDisplayAPPW(); // OLED display
extern int32_t distance_total;
extern int16_t cbmGenInfoLine;
extern String  cbmDeviceId, cbmWiFiId;

Preferences preferences;  // persistant data storage in NVS flash area
CBMConfig_s CBMConfig;
uint8_t     configServerUptimeMax = 240; // wait time for user connection in seconds (max: 255 seconds)

String ExpertMode = "off";  // switch between ExpertMode=on (Default mode=off) to display extended config page

// We need to specify some content-type mapping, so the resources get delivered with the
// right content type and are displayed correctly in the browser
char contentTypes[][2][32] = {
  {".txt",  "text/plain"},
  {".html",  "text/html"},
  {".png",   "image/png"},
  {".jpg",   "image/jpg"},
  {"", ""}
};

// The HTTPS Server comes in a separate namespace. For easier use, include it here.
using namespace httpsserver;

static const char *const HTTP_GET = "GET";
static const char *const HTTP_POST = "POST";

static const int BYTES_PER_KB = 1024;
static const int BYTES_PER_MB = 1024 * 1024;

static HTTPSServer * server;
static HTTPServer  * insecureServer;

bool configServerWasConnectedViaHttpFlag = false;

bool configServerWasConnectedViaHttp() {
  return configServerWasConnectedViaHttpFlag;
}

void touchConfigServerHttp() {
  configServerWasConnectedViaHttpFlag = true;
}

void eraseConfigData() {
  preferences.begin("CBMpreferences", false);
  preferences.clear();
  delay(5000);
  preferences.end();
  ESP.restart();
}

void readConfigurationData() {
  Log.info(F("readConfigurationData"));
  // max length of preferences name 15 characters
  String wifi_ssid, wifi_password, ap_password;
  String wheel_size, dynamo_poles, distance_total, tx_interval, deep_sleep;
  String project_url, Language, Timezone, ntpServer, gmtOffset, daylightOffset, hb_apiurl, hb_authkey;

  preferences.begin("CBMpreferences", false);

  wifi_ssid = preferences.getString("wifi_ssid", "freifunk.berlin.net");
  wifi_password = preferences.getString("wifi_password", "");
  ap_password = preferences.getString("ap_password", "");    // password for the local access point for configuration
  project_url = preferences.getString("project_url", "https://sai-lab.de");
  wheel_size = preferences.getString("wheel_size", "");
  dynamo_poles = preferences.getString("dynamo_poles", "28");
  distance_total = preferences.getString("distance_total", "");
  tx_interval = preferences.getString("tx_interval", "900"); // transmit interval if cycling is activ
  deep_sleep = preferences.getString("deep_sleep", "3600");  // sent every hour data, even if not cycling
  Language = preferences.getString("Language", "de");
  Timezone = preferences.getString("Timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
  ntpServer = preferences.getString("ntpServer", "europe.pool.ntp.org");
  gmtOffset = preferences.getString("gmtOffset", "0");
  daylightOffset = preferences.getString("daylightOffset", "0");
  hb_apiurl = preferences.getString("hb_apiurl", ""); // URL MUST end with a trailing /
  hb_authkey = preferences.getString("hb_authkey", "");

  // save preference variables into global object
  wifi_ssid.toCharArray(CBMConfig.WIFI_SSID,wifi_ssid.length()+1);
  wifi_password.toCharArray(CBMConfig.WIFI_PASSWORD,wifi_password.length()+1);
  ap_password.toCharArray(CBMConfig.AP_PASSWORD,ap_password.length()+1);
  project_url.toCharArray(CBMConfig.project_url,project_url.length()+1);
  wheel_size.toCharArray(CBMConfig.wheel_size,wheel_size.length()+1);
  dynamo_poles.toCharArray(CBMConfig.dynamo_poles,dynamo_poles.length()+1);
  distance_total.toCharArray(CBMConfig.distance_total,distance_total.length()+1);
  tx_interval.toCharArray(CBMConfig.tx_interval,tx_interval.length()+1);
  deep_sleep.toCharArray(CBMConfig.deep_sleep,deep_sleep.length()+1);
  Language.toCharArray(CBMConfig.Language,Language.length()+1);
  Timezone.toCharArray(CBMConfig.Timezone,Timezone.length()+1);
  ntpServer.toCharArray(CBMConfig.ntpServer,ntpServer.length()+1);
  gmtOffset.toCharArray(CBMConfig.gmtOffset,gmtOffset.length()+1);
  daylightOffset.toCharArray(CBMConfig.daylightOffset,daylightOffset.length()+1);
  hb_apiurl.toCharArray(CBMConfig.hb_apiurl,hb_apiurl.length()+1);
  hb_authkey.toCharArray(CBMConfig.hb_authkey,hb_authkey.length()+1);

  Log.verbose(F("readConfigurationData: print configuration data read from NVM"));
  Log.verbose(F("WIFI_SSID: %s"), CBMConfig.WIFI_SSID);
  Log.verbose(F("WIFI_PASSWORD: %s"), CBMConfig.WIFI_PASSWORD);
  Log.verbose(F("AP_PASSWORD: %s"), CBMConfig.AP_PASSWORD);
  Log.verbose(F("Project URL: %s"), CBMConfig.project_url);
  Log.verbose(F("Wheel size: %s"), CBMConfig.wheel_size);
  Log.verbose(F("Dynamo poles: %s"), CBMConfig.dynamo_poles);
  Log.verbose(F("Distance total: <%s>"), CBMConfig.distance_total);
  Log.verbose(F("TX interval: %s"), CBMConfig.tx_interval);
  Log.verbose(F("Deep sleep time: %s"), CBMConfig.deep_sleep);
  Log.verbose(F("Language: %s"), CBMConfig.Language);
  Log.verbose(F("Timezone: %s"), CBMConfig.Timezone);
  Log.verbose(F("ntpServer: %s"), CBMConfig.ntpServer);
  Log.verbose(F("gmtOffset: %s"), CBMConfig.gmtOffset);
  Log.verbose(F("daylightOffset: %s"), CBMConfig.daylightOffset);
  Log.verbose(F("hb_apiurl: %s"), CBMConfig.hb_apiurl);
  Log.verbose(F("hb_authkey: %s"), CBMConfig.hb_authkey);

  preferences.end();
}

void writeConfigDataWifi()
{
  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  Log.verbose(F("writeConfigDataWifi: print current configuration data from CBMConfig object"));
  Log.verbose(F("WIFI_SSID: %s"), CBMConfig.WIFI_SSID);
  Log.verbose(F("WIFI_PASSWORD: %s"), CBMConfig.WIFI_PASSWORD);

  preferences.begin("CBMpreferences", false);
  preferences.putString("wifi_ssid", CBMConfig.WIFI_SSID);
  preferences.putString("wifi_password", CBMConfig.WIFI_PASSWORD);
  preferences.end();
}

void writeConfigDataDistanceTotal()
{
  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  Log.verbose(F("writeConfigDataDistanceTotal: print current configuration data from CBMConfig object"));
  Log.verbose(F("CBMConfig.distance_total: %s"), CBMConfig.distance_total);
  preferences.begin("CBMpreferences", false);
  preferences.putString("distance_total", CBMConfig.distance_total);
  preferences.end();
  // read save configuration data for verification
  Log.verbose(F("writeConfigDataDistanceTotal: read config data for verification"));
  readConfigurationData();
}

void writeConfigDataWheelSize()
{
  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  Log.verbose(F("writeConfigDataWheelSize: print current configuration data from CBMConfig object"));
  Log.verbose(F("CBMConfig.wheel_size: %s"), CBMConfig.wheel_size);
  preferences.begin("CBMpreferences", false);
  preferences.putString("wheel_size", "2500");
  preferences.end();
  // read save configuration data for verification
  Log.verbose(F("writeConfigDataWheelSize: read config data for verification"));
  readConfigurationData();
}

void writeConfigData()
{
  Log.info(F("writeConfigData: entry"));
  Log.verbose(F("writeConfigData: print current configuration data from CBMCOnfig object"));
  Log.verbose(F("WIFI_SSID: %s"), CBMConfig.WIFI_SSID);
  Log.verbose(F("WIFI_PASSWORD: %s"), CBMConfig.WIFI_PASSWORD);
  Log.verbose(F("AP_PASSWORD: %s"), CBMConfig.AP_PASSWORD);
  Log.verbose(F("Project URL: %s"), CBMConfig.project_url);
  Log.verbose(F("Wheel size: %s"), CBMConfig.wheel_size);
  Log.verbose(F("Dynamo poles: %s"), CBMConfig.dynamo_poles);
  Log.verbose(F("Distance total: %s"), CBMConfig.distance_total);
  Log.verbose(F("Tx interval: %s"), CBMConfig.tx_interval);
  Log.verbose(F("Deep sleep time: %s"), CBMConfig.deep_sleep);
  Log.verbose(F("Language: %s"), CBMConfig.Language);
  Log.verbose(F("Timezone: %s"), CBMConfig.Timezone);
  Log.verbose(F("ntpServer: %s"), CBMConfig.ntpServer);
  Log.verbose(F("gmtOffset: %s"), CBMConfig.gmtOffset);
  Log.verbose(F("daylightOffset: %s"), CBMConfig.daylightOffset);
  Log.verbose(F("hb_apiurl: %s"), CBMConfig.hb_apiurl);
  Log.verbose(F("hb_authkey: %s"), CBMConfig.hb_authkey);

  preferences.begin("CBMpreferences", false);
  preferences.putString("project_url", CBMConfig.project_url);
  preferences.putString("wheel_size", CBMConfig.wheel_size);
  preferences.putString("dynamo_poles", CBMConfig.dynamo_poles);
  preferences.putString("distance_total", CBMConfig.distance_total);
  preferences.putString("tx_interval", CBMConfig.tx_interval);
  preferences.putString("deep_sleep", CBMConfig.deep_sleep);
  preferences.putString("Language", CBMConfig.Language);
  preferences.putString("Timezone", CBMConfig.Timezone);
  preferences.putString("ntpServer", CBMConfig.ntpServer);
  preferences.putString("gmtOffset", CBMConfig.gmtOffset);
  preferences.putString("daylightOffset", CBMConfig.daylightOffset);
  preferences.putString("hb_apiurl", CBMConfig.hb_apiurl);
  preferences.putString("hb_authkey", CBMConfig.hb_authkey);
  preferences.end();

  // read save configuration data for verification
  Log.verbose(F("writeConfigData: read config data for verification"));
  readConfigurationData();

}

// Create an SSL certificate object from the files included above
SSLCert CBMcert = SSLCert(
  cbm_crt_DER, cbm_crt_DER_len,
  cbm_key_DER, cbm_key_DER_len
);

// Style
static const String style =
  "<style>"
  "#file-input,input, button {width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px;}"
  "input, button, a.back {background:#f1f1f1;border:0;padding:0;text-align:center;}"
  "body {background:#3498db;font-family:sans-serif;font-size:12px;color:#777}"
  "#file-input {padding:0 5px;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
  "#bar,#prgbar {background-color:#f1f1f1;border-radius:10px}"
  "#bar {background-color:#3498db;width:0%;height:10px}"
  "form {background:#fff;max-width:400px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
  ".btn {background:#3498db;color:#fff;cursor:pointer}"
  "h1,h2, h3 {padding:0;margin:0;}"
  "h3 {padding:10px 0;margin-top:10px;margin-bottom:10px;border-top:3px solid #3498db;border-bottom:3px solid #3498db;}"
  "h1 a {color:#777}"
  "h2 {margin-top:5px}"
  "hr { border-top:1px solid #CCC;margin-left:10px;margin-right:10px;}"
  ".deletePrivacyArea, a.back {color: black; text-decoration: none; font-size: x-large;}"
  ".deletePrivacyArea:hover {color: red;}"
  "a.previous {text-decoration: none; display: inline-block; padding: 8px 16px;background-color: #f1f1f1; color: black;border-radius: 50%; font-family: Verdana, sans-serif; font-size: 18px}"
  "a.previous:hover {background-color: #ddd; color: black;}"
  "ul.directory-listing {list-style: none; text-align: left; padding: 0; margin: 0; line-height: 1.5;}"
  "li.directory a {text-decoration: none; font-weight: bold;}"
  "li.file a {text-decoration: none;}"
  "</style>";

static const String previous = "<a href=\"javascript:history.back()\" class='previous'>&#8249;</a>";

static const String header =
  "<!DOCTYPE html>\n"
  "<html lang='en'><head><meta charset='utf-8'/><title>{title}</title>" + style +
  "<link rel='icon' href='data:;base64,iVBORw0KGgo=' />"
  "<script>"
  "window.onload = function() {"
  "  if (window.location.pathname == '/') {"
  "    document.querySelectorAll('.previous')[0].style.display = 'none';"
  "  } else {"
  "    document.querySelectorAll('.previous')[0].style.display = '';"
  "  }"
  "}"
  "</script></head><body>"
  ""
  "<form action='{action}' method='POST'>"
  "<h1><a href='/'>Cargobikometer</a></h1>"
  "<h2>{subtitle}</h2>"
  "<p>Firmware version: {version}</p>"
  + previous;

static const String footer = "</form></body></html>";

// #########################################
// Upload form
// #########################################
static const String xhrUpload =   
  "<input type='file' name='upload' id='file' accept='{accept}'>"
  "<label id='file-input' for='file'>Choose file...</label>"
  "<input id='btn' type='submit' class=btn value='Upload'>"
  "<br><br>"
  "<div id='prg'></div>"
  "<br><div id='prgbar'><div id='bar'></div></div><br>" // </form>"
  "<script>"
  ""
  "function hide(x) { x.style.display = 'none'; }"
  "function show(x) { x.style.display = 'block'; }"
  ""
  "hide(document.getElementById('file'));"
  "hide(document.getElementById('prgbar'));"
  "hide(document.getElementById('prg'));"
  ""
  "var fileName = '';"
  "document.getElementById('file').addEventListener('change', function(e){"
  "fileNameParts = e.target.value.split('\\\\');"
  "fileName = fileNameParts[fileNameParts.length-1];"
  "console.log(fileName);"
  "document.getElementById('file-input').innerHTML = fileName;"
  "});"
  ""
  "document.getElementById('btn').addEventListener('click', function(e){"
  "e.preventDefault();"
  "if (fileName == '') { alert('No file chosen'); return; }"
  "console.log('Start upload...');"
  ""
  "var form = document.getElementsByTagName('form')[0];"
  "var data = new FormData(form);"
  "console.log(data);"
  //https://developer.mozilla.org/en-US/docs/Web/API/FormData/values
  "for (var v of data.values()) { console.log(v); }"
  ""
  "hide(document.getElementById('file-input'));"
  "hide(document.getElementById('btn'));"
  "show(document.getElementById('prgbar'));"
  "show(document.getElementById('prg'));"
  ""
  "var xhr = new XMLHttpRequest();"
  "xhr.open( 'POST', '{method}', true );"
  "xhr.onreadystatechange = function(s) {"
  "console.log(xhr.responseText);"
  "if (xhr.readyState == 4 && xhr.status == 200) {"
  "document.getElementById('prg').innerHTML = xhr.responseText;"
  "} else if (xhr.readyState == 4 && xhr.status == 500) {"
  "document.getElementById('prg').innerHTML = 'Upload error:' + xhr.responseText;"
  "} else {"
  "document.getElementById('prg').innerHTML = 'Unknown error';"
  "}"
  "};"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = Math.round((evt.loaded * 100) / evt.total);"
  "if(per == 100) document.getElementById('prg').innerHTML = 'Updating...';"
  "else document.getElementById('prg').innerHTML = 'Upload progress: ' + per + '%';"
  "document.getElementById('bar').style.width = per + '%';"
  "}"
  "}, false);"
  "xhr.send( data );"
  "});" // btn click
  ""
  "</script>";

// #########################################
// Navigation
// #########################################
static const String navigationIndex =
  "<h3>Settings</h3>"
  "<input type=button onclick=\"window.location.href='/settings/configmode'\" class=btn value='Configuration Mode'>"
  "<input type=button onclick=\"window.location.href='/settings/general'\" class=btn value='General'>"
  "<input type=button onclick=\"window.location.href='/settings/wifi'\" class=btn value='Wifi'>"

   "<h3>Maintenance</h3>"
  "<input type=button onclick=\"window.location.href='/update'\" class=btn value='Update Firmware'>"
  "<input type=button onclick=\"window.location.href='/about'\" class=btn value='About'>"
  "<input type=button onclick=\"window.location.href='/reboot'\" class=btn value='Reboot and activate configuration'>"
  "{dev}";

static const String httpsRedirect =
  "<h3>HTTPS</h3>"
  "You need to access the device for configuration via secure https. If not done already, you also need to "
  "accept the self signed certificate after pressing 'Goto https'."
  "<input type=button onclick=\"window.location.href='https://{host}'\" class=btn value='Goto https'>";

// Server Index Page
static const String uploadIndex = "<h3>Update</h3>";

// #########################################
// Reboot
// #########################################
static const String rebootIndex =
  "<h3>Device reboots now.</h3>";

// #########################################
// Wifi
// #########################################
static const String wifiSettingsIndex =
  "<script>"
  "function resetPassword() { document.getElementById('pass').value = ''; }"
  "</script>"
  "<h3>Settings</h3>"
  "SSID"
  "<input name=ssid placeholder='ssid' type='text' required maxlength='30' value='{ssid}'>"
  "<small>max length: 30</small>"
  "<hr>"
  "Password"
  "<input id=pass name=pass placeholder='password' type='Password' maxlength='30' value='{password}' onclick='resetPassword()'>"
  "<small>not required (e.g. for Freifunk/Germany) - max length: 30</small>"
  "<input type=submit class=btn value=Save>";

// #########################################
// Config mode index
// #########################################
static const String ConfigModeIndex =
  "<h3>Configuration mode</h3>"
  "<label for=''>ExpertMode </label>"
  "<select name='ExpertMode' id='ExpertMode'>" 
  "<option value='{ExpertMode}' selected>current: {ExpertMode}</option>"
  "<option value='on' >on</option>"
  "<option value='off' >off</option>"
  "</select>"
  "<br>"
  "<small>ExpertMode=on switches to advanced configuration options in menue General for the current session</small>"
  "<hr>"
  "<input type=submit class=btn value=Change>";

// #########################################
// Config
// #########################################
static const String configIndexDefault =
  "<script>"
  "function resetAPIKey() { document.getElementById('tr_apikey').value = ''; }"
  "</script>"

"<h3>Bicycle</h3>"
 "<label for='wheel_size'>Wheel size in mm </label>"
  "<select name='wheel_size' id='wheel_size'>" 
  "<option value='{wheel_size}' selected>current: {wheel_size}</option>"
  "<option value='1272'          >47-305/16x1.75x2 - 1272 mm</option>"
  "<option value='1590'          >47-406/20x1.75x2 - 1590 mm</option>"
  "<option value='1948'          >37-540/24x1 3/8A - 1948 mm</option>" 
  "<option value='1907'          >47-507/24x1.75x2 - 1907 mm</option>" 
  "<option value='1973'          >23-571/26x1      - 1973 mm</option>" 
  "<option value='2026'          >40-559/26x1.5    - 2026 mm</option>"
  "<option value='2051'          >44-559/26x1.6    - 2051 mm</option>"
  "<option value='2070'          >47-559/26x1.75x2 - 2070 mm</option>"
  "<option value='2089'          >50-559/26x1.9    - 2089 mm</option>"
  "<option value='2114'          >54-559/26x2.00   - 2114 mm</option>"
  "<option value='2133'          >57-559/26x2.125  - 2133 mm</option>"
  "<option value='2105'          >37-559/26x1 3/8  - 2105 mm</option>"
  "<option value='2086'          >37-584/26x1 3/8x1/2 - 2086 mm</option>"
  "<option value='1954'          >20-571/26x3/4    - 1973 mm</option>"
  "<option value='2199'          >32-630/27x1 1/4  - 2199 mm</option>"
  "<option value='2174'          >28-630/27x1 1/4 Fifty - 2174 mm</option>"
  "<option value='2224'          >40-622/28x1.5    - 2224 mm</option>"
  "<option value='2268'          >47-622/28x1.75   - 2268 mm</option>"
  "<option value='2265'          >40-635/28x1 1/2  - 2265 mm</option>"
  "<option value='2205'          >37-622/28x1 3/8x1 5/8   - 2205 mm</option>" 
  "<option value='2102'          >18-622/700x18C   - 2102 mm</option>"  
  "<option value='2114'          >20-622/700x20C   - 2114 mm</option>" 
  "<option value='2133'          >23-622/700x23C   - 2133 mm</option>" 
  "<option value='2146'          >25-622/700x25C   - 2146 mm</option>"
  "<option value='2149'          >28-622/700x28C   - 2149 mm</option>"
  "<option value='2174'          >32-622/700x32C   - 2174 mm</option>" 
  "<option value='2205'          >37-622/700x35C   - 2205 mm</option>" 
  "<option value='2224'          >40-622/700x40C   - 2224 mm</option>" 
  "<option value='222'           >CargoBikeCity-Station - 222 mm</option>" 
  "</select>"
  "<hr>"

 "<label for='dynamo_poles'>Hub dynamo poles </label>"
  "<select name='dynamo_poles' id='dynamo_poles'>" 
  "<option value='{dynamo_poles}' selected>current: {dynamo_poles}</option>"
  "<option value='28'            >Shimano / 28 poles</option>"
  "<option value='26'            >SON / 26 poles</option>"
  "</select>"
  "<hr>"

  "Distance total in km"
  "<input name=distance_total placeholder='distance_total' type='number' required step='1' min='0' max='999999' value='{distance_total}'>"
  "<small>Set or reset the total distance in km travelled</small>"

"<h3>System</h3>"
  "<label for='Language'>Language </label>"
  "<select name='Language' id='Language'>" 
  "<option value='{Language}' selected>current: {Language}</option>"
  "<option value='DE' >DE</option>"
  "<option value='EN' >EN</option>"
  "<option value='NL' >NL</option>"
  "<option value='FR' >FR</option>"
  "<option value='ES' >ES</option>"
  "<option value='SL' >SL</option>"
  "</select>"
  "<hr>"

  // see: https://sites.google.com/a/usapiens.com/opnode/time-zones
  "<label for='Timezone'>Timezone </label>"
  "<select name='Timezone' id='Timezone'>" 
  "<option value='{Timezone}' selected>current: {Timezone}</option>"
  "<option value='CET-1CEST,M3.5.0,M10.5.0/3'      >Amsterdam, Netherlands</option>"
  "<option value='EET-2EEST-3,M3.5.0/3,M10.5.0/4'  >Athens, Greece</option>"
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Barcelona, Spain</option>"
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Berlin, Germany</option>"
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3:' >Brussels, Belgium</option>"
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Budapest, Hungary</option>"
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Copenhagen, Denmark</option>"
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Dublin, Ireland</option>"
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Geneva, Switzerland</option>"
  "<option value='EET-2EEST-3,M3.5.0/3,M10.5.0/4'  >Helsinki, Finland</option>"
  "<option value='EET-2EEST,M3.5.0/3,M10.5.0/4'    >Kyiv, Ukraine</option>"  
  "<option value='WET-0WEST-1,M3.5.0/1,M10.5.0/2'  >Lisbon, Portugal</option>" 
  "<option value='GMT+0BST-1,M3.5.0/1,M10.5.0/2'   >London, Great Britain</option>" 
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Madrid, Spain</option>" 
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Oslo, Norway</option>" 
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Paris, France</option>" 
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Prague, Czech Republic</option>" 
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Roma, Italy</option>" 
  "<option value='MSK-3MSD,M3.5.0/2,M10.5.0/3'     >Moscow, Russia</option>"  
  "<option value='EET-2EEST-3,M3.5.0/3,M10.5.0/4'  >Sofia, Bulgaria</option>" 
  "<option value='MST-3MDT,M3.5.0/2,M10.5.0/3'     >St.Petersburg, Russia</option>"  
  "<option value='CET-1CEST-2,M3.5.0/2,M10.5.0/3'  >Stockholm, Sweden</option>" 
  "<option value='EET-2EEST-3,M3.5.0/3,M10.5.0/4'  >Tallinn, Estonia</option>"   
  "<option value='CET-1CEST,M3.5.0,M10.5.0/3'      >Warsaw, Poland</option>" 
   
  // USA AND CANADA
  "<option value='HAW10'                           >Hawaii Time</option>"
  "<option value='AKST9AKDT,M3.2.0,M11.1.0'        >Alaska Time</option>"
  "<option value='PST8PDT,M3.2.0,M11.1.0'          >Pacific Time</option>"
  "<option value='MST7MDT,M3.2.0,M11.1.0'          >Mountain Time</option>"
  "<option value='MST7'                            >Mountain Time (Arizona, no DST)</option>"
  "<option value='CST6CDT,M3.2.0,M11.1.0'          >Central Time</option>"
  "<option value='EST5EDT,M3.2.0,M11.1.0'          >Eastern Time</option>"
  "<option value='AST4ADT'                         >Atlantic Time</option>"

  // AUSTRALIA
  "<option value='AEST-10AEDT-11,M10.5.0/2,M4.1.0/3'      >Melbourne,Canberra,Sydney</option>"
  "<option value='AWST-8AWDT-9,M10.5.0,M3.5.0/3'          >Perth</option>"
  "<option value='AEST-10'                                >Brisbane</option>"
  "<option value='ACST-9:30ACDT-10:30,M10.5.0/2,M4.1.0/3' >Adelaide</option>"
  "<option value='ACST-9:30'                              >Darwin</option>"
  "<option value='AEST-10AEDT-11,M10.1.0/2,M4.1.0/3'      >Hobart</option>"

  // NEW ZEALAND
  "<option value='NZST-12NZDT-13,M10.1.0/2,M3.3.0/3'      >Auckland, Wellington</option>"

  // ASIA
  "<option value='WIB-7'                           >Jakarta</option>"
  "<option value='SGT-8'                           >Singapore</option>"
  "<option value='HKT-8'                           >Hong Kong</option>"
  "<option value='ULAT-8ULAST,M3.5.0/2,M9.5.0/2'   >Ulaanbaatar, Mongolia</option>"

  // CENTRAL AND SOUTH AMERICA
  "<option value='BRST+3BRDT+2,M10.3.0,M2.3.0'     >Brazil, São Paulo</option>"
  "<option value='UTC+5'                           >Colombia</option>"
  "<option value='UTC+3'                           >Argentina</option>"
  "<option value='UTC+6'                           >Central America</option>"

  "</select>"
  "<hr>"

  "<label for='ntpServer'>ntpServer </label>"
  "<select name='ntpServer' id='ntpServer'>" 
  "<option value='{ntpServer}' selected>current: {ntpServer}</option>"
  "<option value='europe.pool.ntp.org'            >Europe</option>"
  "<option value='north-america.pool.ntp.org'     >North-America</option>"
  "<option value='oceania.pool.ntp.org'           >Oceania</option>"
  "<option value='asia.pool.ntp.org'              >Asia</option>"
  "<option value='pool.ntp.org'                   >global: pool.ntp.org</option>"
  "</select>"
  "<hr>"

  "TX interval"
  "<input name=tx_interval placeholder='tx_interval' type='number' required step='1' min='60' max='99999' value='{tx_interval}'>"
  "<small>how often data should be send in seconds - allowed value: 60-99999</small>"
  "<hr>"

   "deep sleep time"
  "<input name=deep_sleep placeholder='deep_sleep' type='number' required step='1' min='60' max='99999'  value='{deep_sleep}'>"
  "<small>how many seconds to sleep - allowed value: 60-99999</small>"
  "<hr>"

  "project URL (optional)"
  "<input name=project_url placeholder='project_url' type='text' maxlength='50' value='{project_url}'>"
  "<small>your own project page URL - max length: 50</small>"

  "<input type=submit class=btn value=Save>";

static const String configIndexExpert =
  "<script>"
  "function resetHBAUTHKey() { document.getElementById('hb_authkey').value = ''; }"
  "</script>"

  "<hr>"
  "Timezone"
  "<input name=Timezone placeholder='Timezone  ' type='text' required maxlength='39' value='{Timezone}'>"
  "<small>see: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv "
  "or https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html "
  "or https://sites.google.com/a/usapiens.com/opnode/time-zones "
  "- max length: 39</small>"
  "<hr>"

  "ntpServer"
  "<input name=ntpServer placeholder='ntpServer ' type='text' required maxlength='39' value='{ntpServer}'>"
  "<small>choose from https://www.ntppool.org "
  "- max length: 39</small>"

  "<hr>"
  "gmtOffset (in seconds)" 
  "<input name=gmtOffset placeholder='gmtOffset' type='text' required maxlength='39' pattern='[0-9]' value='{gmtOffset}'>"
  "<small>UK normal time is GMT, so GMT Offset is 0, for DE (+1hr) use 3600, "
  "for US (-5hrs) is typically -18000, AU is typically (+8hrs) 28800 "
  "- max length: 39</small>"
  "<hr>"

  "daylightOffset (in seconds)"
  "<input name=daylightOffset placeholder='daylightOffset' type='text' required maxlength='39' pattern='[0-9]' value='{daylightOffset}'>"
  "<small>In the UK DST is +1hr (3600-secs), other countries (e.g. DE) may use 2hrs (7200-secs) "
  "or 30-mins (1800-secs) or 5.5hrs (19800-secs); ahead of GMT use + offset behind - offset "
  "- max length: 39</small>"

  "<h3>Heartbeat server (optional)</h3>"
  "URL"
  "<input name=hb_apiurl placeholder='hb_apiurl' type='text' maxlength='50' value='{hb_apiurl}'>"
  "<small>server which receives heartbeat messages from the device - max length: 50 </small>"
  "<hr>"

  "Authentication key"
  "<input name=hb_authkey placeholder='hb_authkey' type='password'  maxlength='70' value='{hb_authkey}' onclick='resetHBAUTHKey()'>"
  "<small>key for http basic authentication (Base64) to access the heartbeat server - max length: 70</small>"

  "<input type=submit class=btn value=Save>";

String encodeForXmlAttribute(const String &text) {
  String result(text);
  result.replace("&", "&amp;");
  result.replace("<", "&lt;");
  result.replace(">", "&gt;");
  result.replace("'", "&#39;");
  result.replace("\"", "&#34;");
  return result;
}

String encodeForXmlText(const String &text) {
  String result(text);
  result.replace("&", "&amp;");
  result.replace("<", "&lt;");
  return result;
}

static String replaceHtml(String &body, const String &key, const String &value) {
  String str(body);
  str.replace(key, encodeForXmlAttribute(value));
  return str;
}

static String keyValue(const String& key, const String& value, const String& suffix = "") {
  return "<b>" + encodeForXmlText(key) + ":</b> " + value + suffix + "<br />";
}

static void handleNotFound(HTTPRequest * req, HTTPResponse * res);
static void handleIndex(HTTPRequest * req, HTTPResponse * res);
static void handleAbout(HTTPRequest * req, HTTPResponse * res);
static void handleReboot(HTTPRequest * req, HTTPResponse * res);
static void handleWifi(HTTPRequest * req, HTTPResponse * res);
static void handleWifiSave(HTTPRequest * req, HTTPResponse * res);
static void handleConfigMode(HTTPRequest * req, HTTPResponse * res);
static void handleConfigModeSave(HTTPRequest * req, HTTPResponse * res);
static void handleConfig(HTTPRequest * req, HTTPResponse * res);
static void handleConfigSave(HTTPRequest * req, HTTPResponse * res);
static void handleFirmwareUpdate(HTTPRequest * req, HTTPResponse * res);
static void handleFirmwareUpdateAction(HTTPRequest * req, HTTPResponse * res);
static void handleHttpsRedirect(HTTPRequest *req, HTTPResponse *res);
// not used currently static void accessFilter(HTTPRequest * req, HTTPResponse * res, std::function<void()> next);

String getIp() {
  if (WiFiClass::status() != WL_CONNECTED) {
    return WiFi.softAPIP().toString();
  } else {
    return WiFi.localIP().toString();
  }
}

bool CreateWifiSoftAP() {
  bool SoftAccOK;
  WiFi.disconnect();
  Log.info(F("Initalize SoftAP "));
  String APName = cbmWiFiId;
  String APPassword = CBMConfig.AP_PASSWORD;

  if (APPassword.length() < 9) {
    // Generate a new random password
    char defaultAPPassword[11];
    snprintf(defaultAPPassword, 10, "%09u", esp_random() % 1000000000);
    APPassword = String(defaultAPPassword);
    Log.verbose(F("APPassword generated: %s"), APPassword);

    // Store the password into NVS so it does not change
    preferences.begin("CBMpreferences", false);
    APPassword.toCharArray(CBMConfig.AP_PASSWORD,APPassword.length()+1);
    preferences.putString("ap_password", CBMConfig.AP_PASSWORD);
    // read saved value for verification
    Log.verbose(F("Read saved ap_password from NVS: %s"), preferences.getString("ap_password", ""));
    // Close the Preferences
    preferences.end();

    // show the password on DISPLAY to the user only one time
    showConfigServerDisplayAPPW();
  } 
  
  SoftAccOK  =  WiFi.softAP(APName.c_str(), APPassword.c_str());
  delay(2000); // Without delay I've seen the IP address blank
  /* Soft AP network parameters */
  IPAddress apIP(172, 20, 0, 1);
  IPAddress netMsk(255, 255, 255, 0);

  WiFi.softAPConfig(apIP, apIP, netMsk);
  
  if (SoftAccOK) {
    Log.info("AP successful.");
  } else {
    Log.error(F("ERROR: soft access point for configuration web server "));
    Log.error(F("ERROR: name= %s"), APName.c_str());
    Log.error(F("ERROR: password= %s"), APPassword.c_str());
  }
  return SoftAccOK;
}

String createPage(const String& content, const String& additionalContent = "") {
  configServerWasConnectedViaHttpFlag = true;
  String result;
  result += header;
  result += content;
  result += additionalContent;
  result += footer;

  return result;
}

void sendHtml(HTTPResponse * res, String& data) {
  res->setHeader("Content-Type", "text/html");
  res->print(data);
}

void sendHtml(HTTPResponse * res, const char * data) {
  res->setHeader("Content-Type", "text/html");
  res->print(data);
}

void sendRedirect(HTTPResponse * res, String location) {
  res->setHeader("Location", location.c_str());
  res->setStatusCode(302);
  res->finalize();
}

String replaceDefault(String html, const String& subTitle, const String& action = "#") {
  configServerWasConnectedViaHttpFlag = true;
  html = replaceHtml(html, "{title}",cbmWiFiId + " - " + subTitle);
  html = replaceHtml(html, "{version}", CBMVersion);
  html = replaceHtml(html, "{subtitle}", subTitle);
  html = replaceHtml(html, "{action}", action);

  String ip;
  if (WiFiGenericClass::getMode() == WIFI_MODE_STA) {
    ip = WiFi.localIP().toString();
  } else {
    ip = WiFi.softAPIP().toString();
  }
  
  return html;
}

std::vector<std::pair<String,String>> extractParameters(HTTPRequest *req) {
  Log.info(F("Extracting parameters"));
  std::vector<std::pair<String,String>> parameters;
  if (String(req->getHeader("Content-Type").c_str()).startsWith("application/x-www-form-urlencoded")) {
    HTTPURLEncodedBodyParser parser(req);
    while(parser.nextField()) {
      std::pair<String,String> data;
      data.first = String(parser.getFieldName().c_str());
      data.second = String();
      while (!parser.endOfField()) {
        char buf[513];
        size_t readLength = parser.read((uint8_t *)buf, 512);
        buf[readLength] = 0;
        data.second += String(buf);
      }
      Log.info(F("Http Parameter %s = %s"), data.first.c_str(), data.second.c_str());
      parameters.push_back(data);
    }
  } else {
    Log.error(F("ERROR: Unexpected content type: %s"), req->getHeader("Content-Type").c_str());
  }
  return parameters;
}

static String getParameter(const std::vector<std::pair<String,String>> &params, const String& name, const String&  def = "") {
  for (auto param : params) {
    if (param.first == name) {
      return param.second;
    }
  }
  return def;
}

String toScaledByteString(uint32_t size) {
  String result;
  if (size <= BYTES_PER_KB * 10) {
    result = String(size) + "b";
  } else if (size <= BYTES_PER_MB * 10) {
    result = String(size / BYTES_PER_KB) + "kb";
  } else {
    result = String(size / BYTES_PER_MB) + "mb";
  }
  return result;
}

static void handleIndex(HTTPRequest *, HTTPResponse * res) {
// ###############################################################
// ### Index ###
// ###############################################################
  String html = createPage(navigationIndex);
  html = replaceDefault(html, "Navigation");
#ifdef DEVELOP
  html.replace("{dev}", development);
#else
  html.replace("{dev}", "");
#endif
  sendHtml(res, html);
}

static void handleNotFound(HTTPRequest * req, HTTPResponse * res) {
  // Discard request body, if we received any
  // We do this, as this is the default node and may also server POST/PUT requests
  req->discardRequestBody();

  // Set the response status
  res->setStatusCode(404);
  res->setStatusText("Not Found");

  // Set content type of the response
  res->setHeader("Content-Type", "text/html");

  // Write a tiny HTTP page
  res->println("<!DOCTYPE html>");
  res->println("<html>");
  res->println("<head><title>Not Found</title></head>");
  res->println("<body><h1>404 Not Found</h1><p>The requested resource was not found on this server.</p></body>");
  res->println("</html>");
}

static void handleReboot(HTTPRequest *, HTTPResponse * res) {
  String html = createPage(rebootIndex);
  html = replaceDefault(html, "Reboot");
  sendHtml(res, html);
  res->finalize();
  delay(1000);
  ESP.restart();
};

static void handleAbout(HTTPRequest *, HTTPResponse * res) {
  res->setHeader("Content-Type", "text/html");
  res->print(replaceDefault(header, "About"));
  String page;
  
  res->print("<h3>ESP32</h3>"); // SPDIFF
  page += keyValue("Heap size", toScaledByteString(ESP.getHeapSize()));
  page += keyValue("Free heap", toScaledByteString(ESP.getFreeHeap()));
  page += keyValue("Min. free heap", toScaledByteString(ESP.getMinFreeHeap()));
  String chipId = String((uint32_t) ESP.getEfuseMac(), HEX) + String((uint32_t) (ESP.getEfuseMac() >> 32), HEX);
  chipId.toUpperCase();
  page += keyValue("Chip id", chipId);
  page += keyValue("IDF Version", esp_get_idf_version());
  page += keyValue("cbmDeviceId", cbmDeviceId);
  page += keyValue("Build date:", BUILD_DATE);

  res->print(page);
  res->print(footer);
}

static void handleConfigMode(HTTPRequest *, HTTPResponse * res) {
  String html = createPage(ConfigModeIndex);
  html = replaceDefault(html, "Configuration Mode", "/settings/configmode/action");

  // Form data
  html = replaceHtml(html, "{ExpertMode}", ExpertMode);
  sendHtml(res, html);
};

static void handleConfigModeSave(HTTPRequest * req, HTTPResponse * res) {
  const auto params = extractParameters(req);
  ExpertMode = getParameter(params, "ExpertMode");
  Log.info(F("Configuration ExpertMode set to: %s"), ExpertMode);
  sendRedirect(res, "/settings/configmode");
}

static void handleWifi(HTTPRequest *, HTTPResponse * res) {
  String html = createPage(wifiSettingsIndex);
  html = replaceDefault(html, "WiFi", "/settings/wifi/action");

  // Form data
  html = replaceHtml(html, "{ssid}", CBMConfig.WIFI_SSID);
  if (strlen(CBMConfig.WIFI_PASSWORD) > 0 ) {
    html = replaceHtml(html, "{password}", "******");
  } else {
    html = replaceHtml(html, "{password}", "");
  }
  sendHtml(res, html);
};

static void handleWifiSave(HTTPRequest * req, HTTPResponse * res) {
  const auto params = extractParameters(req);
  const auto ssid = getParameter(params, "ssid");
  if (ssid) {
    ssid.toCharArray(CBMConfig.WIFI_SSID,ssid.length()+1);
  }
  const auto password = getParameter(params, "pass");
  if (password != "******") {
    password.toCharArray(CBMConfig.WIFI_PASSWORD,password.length()+1);
  }

  Log.info(F("WIFI_SSID set to: %s"), CBMConfig.WIFI_SSID);
  Log.info(F("Save Config to NVS RAM ..."));
  writeConfigDataWifi();

  sendRedirect(res, "/settings/wifi");
}

static void handleConfig(HTTPRequest *, HTTPResponse * res) {
  String html;
  Log.info(F("handleConfig: Entry"));
  if ( ExpertMode == "on" ) {
    html = createPage(configIndexExpert);
    html = replaceDefault(html, "General", "/settings/general/action");
  } else {
    html = createPage(configIndexDefault);
    html = replaceDefault(html, "General", "/settings/general/action");
  }

  // Form data
  html = replaceHtml(html, "{Language}", CBMConfig.Language);
  html = replaceHtml(html, "{Timezone}", CBMConfig.Timezone);
  html = replaceHtml(html, "{ntpServer}", CBMConfig.ntpServer);
  html = replaceHtml(html, "{wheel_size}", CBMConfig.wheel_size);
  html = replaceHtml(html, "{dynamo_poles}", CBMConfig.dynamo_poles);
  html = replaceHtml(html, "{distance_total}", CBMConfig.distance_total);
  html = replaceHtml(html, "{tx_interval}", CBMConfig.tx_interval);
  html = replaceHtml(html, "{deep_sleep}", CBMConfig.deep_sleep);
  html = replaceHtml(html, "{project_url}", CBMConfig.project_url);

  if ( ExpertMode == "on" ) {
    html = replaceHtml(html, "{Timezone}", CBMConfig.Timezone);
    html = replaceHtml(html, "{ntpServer}", CBMConfig.ntpServer);
    html = replaceHtml(html, "{gmtOffset}", CBMConfig.gmtOffset);
    html = replaceHtml(html, "{daylightOffset}", CBMConfig.daylightOffset);
    html = replaceHtml(html, "{hb_apiurl}", CBMConfig.hb_apiurl);
    html = replaceHtml(html, "{hb_authkey}", CBMConfig.hb_authkey); 
  }
  Log.info(F("handleConfig: sendHtml ..."));
  sendHtml(res, html);
};

static void handleConfigSave(HTTPRequest * req, HTTPResponse * res) {
  Log.info(F("handleConfigSave: Entry"));
  const auto params = extractParameters(req);
  
  if ( ExpertMode == "off" ) {

    Log.info(F("handleConfigSave: ExpertMode=off"));

    const auto Language = getParameter(params, "Language");
    if (Language) {
      Language.toCharArray(CBMConfig.Language,Language.length()+1);
    }

    const auto Timezone = getParameter(params, "Timezone");
    if (Timezone) {
      Timezone.toCharArray(CBMConfig.Timezone,Timezone.length()+1);
    }

    const auto ntpServer = getParameter(params, "ntpServer");
    if (ntpServer) {
      ntpServer.toCharArray(CBMConfig.ntpServer,ntpServer.length()+1);
    }

    const auto wheel_size = getParameter(params, "wheel_size");
    if (wheel_size) {
      wheel_size.toCharArray(CBMConfig.wheel_size,wheel_size.length()+1);
    }

    const auto dynamo_poles = getParameter(params, "dynamo_poles");
    if (dynamo_poles) {
      dynamo_poles.toCharArray(CBMConfig.dynamo_poles,dynamo_poles.length()+1);
    }

    const auto distance_total = getParameter(params, "distance_total");
    if (distance_total) {
      distance_total.toCharArray(CBMConfig.distance_total,distance_total.length()+1);
    }

    const auto tx_interval = getParameter(params, "tx_interval");
    if (tx_interval) {
      tx_interval.toCharArray(CBMConfig.tx_interval,tx_interval.length()+1);
    }

    const auto deep_sleep = getParameter(params, "deep_sleep");
    if (deep_sleep) {
      deep_sleep.toCharArray(CBMConfig.deep_sleep,deep_sleep.length()+1);
    }

    const auto project_url = getParameter(params, "project_url");
    if (project_url) {
      project_url.toCharArray(CBMConfig.project_url,project_url.length()+1);
    }

  } else {
    Log.info(F("handleConfigSave: ExpertMode=on"));

    const auto Timezone = getParameter(params, "Timezone");
    if (Timezone) {
      Timezone.toCharArray(CBMConfig.Timezone,Timezone.length()+1);
    }

    const auto ntpServer = getParameter(params, "ntpServer");
    if (ntpServer) {
      ntpServer.toCharArray(CBMConfig.ntpServer,ntpServer.length()+1);
    }

    const auto gmtOffset = getParameter(params, "gmtOffset");
      if (gmtOffset) {
        gmtOffset.toCharArray(CBMConfig.gmtOffset,gmtOffset.length()+1);
      }

    const auto daylightOffset = getParameter(params, "daylightOffset");
    if (daylightOffset) {
      daylightOffset.toCharArray(CBMConfig.daylightOffset,daylightOffset.length()+1);
    } 

    const auto hb_apiurl = getParameter(params, "hb_apiurl");
    if (hb_apiurl) {
      hb_apiurl.toCharArray(CBMConfig.hb_apiurl,hb_apiurl.length()+1);
    } 

    const auto hb_authkey = getParameter(params, "hb_authkey");
    if (hb_authkey) {
      hb_authkey.toCharArray(CBMConfig.hb_authkey,hb_authkey.length()+1);
    }         
  }

  Log.info(F("Save Config to NVS RAM ..."));
  writeConfigData();
  sendRedirect(res, "/settings/general");
}

static void handleFirmwareUpdate(HTTPRequest *, HTTPResponse * res) {
  String html = createPage(uploadIndex, xhrUpload);
  html = replaceDefault(html, "Update Firmware");
  html = replaceHtml(html, "{method}", "/update");
  html = replaceHtml(html, "{accept}", ".bin");
  sendHtml(res, html);
};

static void handleFirmwareUpdateAction(HTTPRequest * req, HTTPResponse * res) {
  HTTPMultipartBodyParser parser(req);
  Update.begin();

  while(parser.nextField()) {
    if (parser.getFieldName() != "upload") {
      Log.info(F("Skipping form data %s type %s filename %s"), parser.getFieldName().c_str(),
            parser.getFieldMimeType().c_str(), parser.getFieldFilename().c_str());
      continue;
    }
    Log.info(F("Got form data %s type %s filename %s"), parser.getFieldName().c_str(),
          parser.getFieldMimeType().c_str(), parser.getFieldFilename().c_str());

    while (!parser.endOfField()) {
      byte buffer[512];
      size_t len = parser.read(buffer, 512);
      Log.info(F("Read data %d"), len);
      if (Update.write(buffer, len) != len) {
        Update.printError(Serial);
      }
    }
    Log.info(F("Done writing firmware image"));
    if (Update.end(true)) { // true to set the size to the current progress
      sendHtml(res, "<h1>Update successful! Device reboots now!</h1>");
      res->finalize();
      delay(5000);
      ESP.restart();
    } else {
      String errorMsg = Update.errorString();
      Log.error(F("Esrc/configServer.cpp:RROR: Update: %s"), errorMsg.c_str());
      res->setStatusCode(400);
      res->setStatusText("Invalid data!");
      res->print("ERROR");
    }
  }
}

static void handleHttpsRedirect(HTTPRequest *req, HTTPResponse *res) {
  String html = createPage(httpsRedirect);
  html = replaceDefault(html, "Https Redirect");
  String linkHost(req->getHTTPHeaders()->getValue("linkHost").c_str());
  // this could be more hardened?
  if (!linkHost || linkHost == "") {
    linkHost = getIp();
  }
  html = replaceHtml(html, "{host}", linkHost);
  sendHtml(res, html);
}

void beginPages() {
  // For every resource available on the server, we need to create a ResourceNode
  // The ResourceNode links URL and HTTP method to a handler function
  server->setDefaultNode(new ResourceNode("", HTTP_GET,  handleNotFound));
  server->registerNode(new ResourceNode("/", HTTP_GET,  handleIndex));
  server->registerNode(new ResourceNode("/about", HTTP_GET,  handleAbout));
  server->registerNode(new ResourceNode("/reboot", HTTP_GET,  handleReboot));
  server->registerNode(new ResourceNode("/settings/wifi", HTTP_GET,  handleWifi));
  server->registerNode(new ResourceNode("/settings/wifi/action", HTTP_POST, handleWifiSave));
  server->registerNode(new ResourceNode("/settings/configmode", HTTP_GET,  handleConfigMode));
  server->registerNode(new ResourceNode("/settings/configmode/action", HTTP_POST,  handleConfigModeSave));
  server->registerNode(new ResourceNode("/settings/general", HTTP_GET,  handleConfig));
  server->registerNode(new ResourceNode("/settings/general/action", HTTP_POST, handleConfigSave));
  server->registerNode(new ResourceNode("/update", HTTP_GET, handleFirmwareUpdate));
  server->registerNode(new ResourceNode("/update", HTTP_POST, handleFirmwareUpdateAction));

  server->setDefaultHeader("Server", std::string("Cargobikometer/") + CBMVersion);

  insecureServer->setDefaultNode(new ResourceNode("", HTTP_GET,  handleHttpsRedirect));
  insecureServer->setDefaultHeader("Server", std::string("Cargobikometer/") + CBMVersion);
}

void startConfigServer() 
{
  uint32_t currentSeconds;
  uint32_t configServerStartTime;

  // read config variables from NVS memory
  readConfigurationData();
  Log.info(F("StartConfigServer: WIFI_SSID read from NVS RAM: %s"), CBMConfig.WIFI_SSID);
  Log.info(F("Start local webserver for configuration ..."));

  // create a local WiFi access point on ESP32 without external connection
  CreateWifiSoftAP();

  server = new HTTPSServer(&CBMcert);
  insecureServer = new HTTPServer();

  // register all pages
  beginPages();

  Log.info(F("Starting HTTPS server..."));
  server->start();
  Log.info(F("Starting HTTP server..."));
  insecureServer->start();

  if (server->isRunning()) {
    Log.info(F("Server ready."));
  }

  // now looping for configServerUptimeMax and wait for user http connection
  currentSeconds = (uint32_t) millis() / 1000;
  configServerStartTime = currentSeconds; 

  while ( ((currentSeconds - configServerStartTime) < configServerUptimeMax) || configServerWasConnectedViaHttpFlag)
  {
    server->loop();
    insecureServer->loop();
    currentSeconds = (uint32_t) millis() / 1000;
  }  
  Log.info(F("ConfigServer closed, now process main loop ..."));
}