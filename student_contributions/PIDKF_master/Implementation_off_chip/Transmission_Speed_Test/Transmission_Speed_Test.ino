uint32_t count, prior_count;
uint32_t prior_msec;
uint32_t count_per_second;
unsigned long start;
unsigned long sample;
unsigned long printtime;

// Uncomment this for boards where "SerialUSB" needed for native port
//#define Serial SerialUSB

void setup() {
  Serial.begin(1000000); // edit for highest baud your board can use
  while (!Serial) ;
  count = 10000000; // starting with 8 digits gives consistent chars/line
  prior_count = count;
  count_per_second = 0;
  prior_msec = millis();
  unsigned long t = micros();
  while(micros()-t<5000000);      //Wait 5s before starting
  start = micros();
  sample = micros();
}

void loop() {
  if(micros()-sample > 106){
    Serial.print("count=");
    Serial.print(count);
    Serial.print(", lines/sec=");
    Serial.println(count_per_second);
    count = count + 1;
    uint32_t msec = millis();
    unsigned long iterationtime = micros();
    // if(count == 10500000){             //Debug to find time Teensy uses for 500000 lines
    //   start = micros()-start;
    //   printtime=micros();
    //   while(true){
    //     if(micros()-printtime>5000000){
    //       Serial.println(start);
    //       printtime=micros();
    //     }
    //   }
    // }
    // if(count == 10000002){             //Debug to find time Teensy uses for one iteration
    //  printtime = micros();
    //  while(true){
    //    if(micros()-printtime>5000000){
    //      Serial.println(iterationtime-start);
    //      printtime = micros();
    //    }
    //  }
    // }
    if (msec - prior_msec > 1000) {
      // when 1 second as elapsed, update the lines/sec count
      prior_msec = prior_msec + 1000;
      count_per_second = count - prior_count;
      prior_count = count;
    }
    sample = micros();
  }
}
