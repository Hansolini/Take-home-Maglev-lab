const byte numChars = 250;
char receivedChars[numChars];
char endMarker = '|';
static byte ndx = 0;
char rc;
unsigned long i = 0;
int recctr = 0;
void setup() 
{
  unsigned long time = millis();
  while(millis()-time<2000);      //Wait 2s before starting.
}

float randomDouble(float minf, float maxf)      //Generate random float within given boundaries
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

void loop(){
  unsigned long time2 = 0;
  bool newData = false;

  time2 = micros();
  
//---------Receive data----------
  while (SerialUSB1.available() > 0 && newData == false) {    
      rc = SerialUSB1.read();
      
      if (rc != endMarker) {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
              ndx = numChars - 1;
          }
      }
      else {
          receivedChars[ndx] = '\0'; // terminate the string
          ndx = 0;
          newData = true;
          recctr++;
      }
  }
//--------Generate data--------
  float states[10] = {randomDouble(-0.05,0.05),randomDouble(-0.05,0.05),randomDouble(-0.05,0.05),randomDouble(-1,1),randomDouble(-1,1),randomDouble(-0.015,0.015),randomDouble(-0.015,0.015),randomDouble(-0.015,0.015),randomDouble(-0.25,0.25),randomDouble(-0.25,0.25)};
//--------Send data---------
  byte * b1 = (byte *) &states;     
  Serial.write(b1,40);
}