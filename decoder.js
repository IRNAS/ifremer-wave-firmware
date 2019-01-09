function Decoder(bytes) {

  var I1 = bytes[0];
  var T1 = bytes[1];
  var T01 = bytes[2];
  var H1 = bytes[3];
  var H01 = bytes[4];
  var AP1 = (bytes[6] << 8) | bytes[5];
  var ACC1 = bytes[7];
  var VBAT = (bytes[9] << 8) | bytes[8];
  var TC1 = bytes[10];
  var TC01 = bytes[11];
  var significant_wh = (bytes[13] << 8) | bytes[12];
  var avearage_wh = (bytes[15] << 8) | bytes[14];
  var average_period = (bytes[17] << 8) | bytes[16];

  return {
    Temperature: T1 + T01 / 100.0,
    Humidity: H1 + H01 / 100.0,
    AirPressure: AP1 /10,
    Acceleration: ACC1,
    CPU_temperature: TC1 + TC01 / 100.0,
    Battery: VBAT /100,
    Info: I1,
    Significant_wave_height: significant_wh,
    Average_wave_height: avearage_wh,
    Average_period: average_period
  };
}
