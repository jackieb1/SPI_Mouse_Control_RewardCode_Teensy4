// Minimal valve + serial isolation test for Teensy 4.0 (Behavior PCB v2.0).
// No ADNS sensor code — proves the reward/serial path independent of sensor init.
// Send a 6-digit command (e.g. "050000") at 57600 baud, line ending = Newline:
//   first 3 digits = valve 1 open duration (ms), last 3 = valve 2 duration (ms).

const int valve1Pin = 22;   // HarveyBehaviorPCB v2: VALVE_1_TRIG
const int valve2Pin = 23;   // HarveyBehaviorPCB v2: VALVE_2_TRIG

unsigned long valve1Start = 0, valve2Start = 0;
unsigned long valve1Dur = 0, valve2Dur = 0;
unsigned int  valve1State = 0, valve2State = 0;

void setup() {
  Serial.begin(57600);
  pinMode(valve1Pin, OUTPUT);
  pinMode(valve2Pin, OUTPUT);
  digitalWrite(valve1Pin, LOW);
  digitalWrite(valve2Pin, LOW);

  while (!Serial && millis() < 4000);
  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
  }

  Serial.println('S');
}

void interpretCommand(String m) {
  m.trim();
  if (m.length() != 6) { Serial.println("#"); return; }
  int d1 = m.substring(0, 3).toInt();
  int d2 = m.substring(3).toInt();
  if (d1 > 0) { valve1Start = millis(); valve1State = 1; valve1Dur = d1; digitalWrite(valve1Pin, HIGH); }
  if (d2 > 0) { valve2Start = millis(); valve2State = 1; valve2Dur = d2; digitalWrite(valve2Pin, HIGH); }
  Serial.println("v1," + String(valve1State) + ",v2," + String(valve2State));
}

void loop() {
  if (valve1State == 1 && millis() - valve1Start > valve1Dur) { digitalWrite(valve1Pin, LOW); valve1State = 0; }
  if (valve2State == 1 && millis() - valve2Start > valve2Dur) { digitalWrite(valve2Pin, LOW); valve2State = 0; }

  static String msg = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') { interpretCommand(msg); msg = ""; }
    else msg += c;
  }
}
