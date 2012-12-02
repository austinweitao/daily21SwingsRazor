void serialComm() {
  if(Serial.available() > 0) {

    int incoming = (int) Serial.read();
    // blink to indicate receiving of message
//    digitalWrite(STATUS_LED_PIN, HIGH);
//    delay(1);
//    digitalWrite(STATUS_LED_PIN, LOW);
//    output_message(incoming);
    
    if(incoming == POLL_START) {
      poll_status = 0;
      poll_counter = 0;
      return;
    }
    if(poll_counter < POLL_LENGTH && poll_status == 0) {
      poll_buffer[poll_counter++] = incoming;
    }
    else if(poll_counter >= POLL_LENGTH && poll_status == 0) {
      poll_decode();
      reset();
    }
  }
}

void poll_decode() {
  // poll_buffer[ id, poll_function, poll_crc, poll_end ]
  

  
  if( poll_buffer[DECODE_ID_BYTE] == id ) {          // meant for this razor-id?
  
    // blink to indicate
//    digitalWrite(STATUS_LED_PIN, HIGH);
//    delay(1);
//    digitalWrite(STATUS_LED_PIN, LOW);
    
    float ov;
    String stringmsg;

    if( poll_buffer[DECODE_CRC_BYTE] == crc8(poll_buffer, 2) ) { // CRC calculated in MAX/MSP on 2 bytes, id + func

      switch(poll_buffer[DECODE_FUNCTION_BYTE]) {      // function number
        case 253:                   // output own ID
          output_message(id);
          break;
        case 252:                   // output pitch
          ov = pitch - offset_pitch;
          output_message(CLIP_DATA(DEG_TO_CHAR((WRAP(TO_DEG(ov))*scale_deg))));
          break;
        case 251:                   // output roll
          ov = roll - offset_roll;
          output_message(CLIP_DATA(DEG_TO_CHAR((WRAP(TO_DEG(ov))*scale_deg))));
          break;
        case 250:                   // output yaw
          ov = yaw - offset_yaw;
          output_message(CLIP_DATA(DEG_TO_CHAR((WRAP(TO_DEG(ov))*scale_deg))));
          break;
        case 249:                   // output all 3 values as floats
          stringmsg = String((unsigned long)((pitch+50)*10000)) + String((unsigned long)((roll+50)*10000)) + String((unsigned long)((yaw+150)*10000));
          output_message( stringmsg );
          break;
        case 248:                   // output data from all sensors (9 floats)
          stringmsg = String((unsigned long)((accel[0]+50000)*100)) + String((unsigned long)((accel[1]+50000)*100)) + String((unsigned long)((accel[2]+50000)*100)) + String((unsigned long)((magnetom[0]+50000)*100)) + String((unsigned long)((magnetom[1]+50000)*100)) + String((unsigned long)((magnetom[2]+50000)*100)) + String((unsigned long)((gyro[0]+50000)*100)) + String((unsigned long)((gyro[1]+50000)*100)) + String((unsigned long)((gyro[2]+50000)*100));
          output_message( stringmsg );
          break;
        case 247:                   // turn sensor reading on (at interval)
          read_mode = 1;
          output_message(read_mode);
          break;
        case 246:                   // turn sensor reading off
          read_mode = 0;
          output_message(read_mode);
          break;
        case 245:                   // set offset
          set_offset();
          output_message(CLIP_DATA(DEG_TO_CHAR(TO_DEG(offset_roll))));
          break;
        case 244:                   // clear offset
          reset_offset();
          output_message(CLIP_DATA(DEG_TO_CHAR(TO_DEG(offset_roll))));
          break;
        case 243:                   // TESTING ONLY: scale_degree value
          scale_deg = 0.66666666;
          output_message(scale_deg);
          break;
        case 242:                   // TESTING ONLY: scale_degree value
          scale_deg = 2.0;
          output_message(scale_deg);
          break;
      } 
    }
  }
  
}

void reset() {
  poll_counter = 0;
}

  
void fakeOutput() {
  // output fake message "/high 27 h1 }"
  char fs[] = "{/amp";
  rs485_write(fs,5);
  rs485_write(id);
  float ov = roll - offset_roll;
  int rrrr = CLIP_DATA(DEG_TO_CHAR((WRAP(TO_DEG(ov))*scale_deg)));
//  fs = "h1";
//  rs485_write(fs,fs.length);
  rs485_write(rrrr);
  rs485_write('}');
  
  char fs1[] = "{/hig";
  rs485_write(fs1,5);
  rs485_write(id);
  char fs2[] = "h1";
  rs485_write(fs2,2);
  rs485_write('}');
  
  
  char fs3[] = "{/var";
  rs485_write(fs3,5);
  rs485_write(id);
  int act = (int) random()*5;
  rs485_write(act);
  int syn = (int) random()*5;
  rs485_write(syn);
  rs485_write(0);
  rs485_write('}');
}

void output_message ( String st ) {
  char msg[5 + st.length()];
  
  msg_buffer[0] = id;
  msg_buffer[1] = st.length();
  
  msg[MSG_START_BYTE] = MSG_START;    // start bit
  msg[MSG_ID_BYTE] = id;           // razor id
  msg[MSG_LENGTH_BYTE] = st.length();            // length of data
  
  int i = 0;
  do{
    msg[MSG_DATA_BYTE+i] = st[i];  // DATA
    msg_buffer[2+i] = st[i];    // for crc
    i++;
  } while(i < st.length());
  
  msg[MSG_CRC_BYTE + st.length() -1] = crc8(msg_buffer, st.length()+2);   // crc bit
  msg[MSG_END_BYTE + st.length() -1] = MSG_END;      // end bit
  rs485_write(msg, 5+st.length());
  
  // blink to indicate sending of message
//  digitalWrite(STATUS_LED_PIN, HIGH);
//  delay(1);
//  digitalWrite(STATUS_LED_PIN, LOW);
      
  // reset sensor timer
  timestamp = millis();
}

//void output_message ( char * buffer, int buffer_length ) {
//  char msg[5+buffer_length];
//  
//  msg_buffer[0] = id;
//  msg_buffer[1] = buffer_length;
//  
//  msg[MSG_START_BYTE] = MSG_START;    // start bit
//  msg[MSG_ID_BYTE] = id;           // razor id
//  msg[MSG_LENGTH_BYTE] = buffer_length;            // length of data
//  
//  int i = 0;
//  do{
//    msg[MSG_DATA_BYTE+i] = buffer[i];  // DATA
//    msg_buffer[2+i] = buffer[i];   // for crc
//    i++;
//  } while(i < buffer_length);
//           
//  msg[MSG_CRC_BYTE + buffer_length -1] = crc8(msg_buffer, buffer_length+2);   // crc bit
//  msg[MSG_END_BYTE + buffer_length -1] = MSG_END;      // end bit
//  rs485_write(msg, 5+buffer_length);
//  
//  // blink to indicate sending of message
////  digitalWrite(STATUS_LED_PIN, HIGH);
////  delay(1);
////  digitalWrite(STATUS_LED_PIN, LOW);
//      
//  // reset sensor timer
//  timestamp = millis();
//}

void output_message( char c ) {
  char msg[6];
  
  msg_buffer[0] = id;
  msg_buffer[1] = 1;
  msg_buffer[2] = c;
  
  msg[MSG_START_BYTE] = MSG_START;    // start bit
  msg[MSG_ID_BYTE] = id;           // razor id
  msg[MSG_LENGTH_BYTE] = 1;            // length of data
  msg[MSG_DATA_BYTE] = c;            // DATA
  msg[MSG_CRC_BYTE] = crc8(msg_buffer,3);            // crc bit
  msg[MSG_END_BYTE] = MSG_END;      // end bit
  rs485_write(msg, 6);
  
  // blink to indicate sending of message
//  digitalWrite(STATUS_LED_PIN, HIGH);
//  delay(1);
//  digitalWrite(STATUS_LED_PIN, LOW);
      
  // reset sensor timer
  timestamp = millis();
}

void output_message( int c ) {
  if(c <= 255 && c >= 0) {
    output_message( (char) c );
  } else {
    // [ TODO: convert int to string ]
  }
}

void output_message( double f ) {
  output_message( (int) f );
}

void output_message( float f ) {
  output_message( (int) f );
}

void rs485_write(int c) {
//  delayMicroseconds(100);
  digitalWrite(RTS_PIN, HIGH); // DE high
  Serial.write(c);
  delayMicroseconds(500);
  digitalWrite(RTS_PIN, LOW); //  RE low
}


void rs485_write(char * buffer, int buffer_length) {
  digitalWrite(RTS_PIN, HIGH); // DE high
  int i = 0;
  do{
    Serial.write(buffer[i++]);
  } while(i < buffer_length);
  delayMicroseconds(500);
  digitalWrite(RTS_PIN, LOW); //  RE low
}

void rs485_write(int * buffer, int buffer_length) {
  digitalWrite(RTS_PIN, HIGH); // DE high
  int i = 0;
  do{
    Serial.write(buffer[i++]);
  } while(i < buffer_length);
  delayMicroseconds(500);
  digitalWrite(RTS_PIN, LOW); //  RE low
}

/*
  Calculates CRC8 for a given buffer
*/

int crc8(int * buffer, int buffer_length) {
  int i,j;
  int c;
  int CRC=0x00;
  for(j = 0; j < buffer_length; j++) {
    c = buffer[j];
    for(i = 0; i<8; i++){
      if(((CRC ^ c) & 0x01)==0){
	CRC >>= 1;
      }
      else{
	CRC ^= 0x18;
	CRC >>= 1;
	CRC |= 0x80;
      }
      c >>= 1;
    }
  }
  if( CRC == MSG_START || CRC == MSG_END || CRC == POLL_START) CRC = 99;
  return CRC;
}
