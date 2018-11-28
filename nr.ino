
const uint64_t pipes[2] = { 0x54CDABCD71LL, 0xAB4d52687CLL };              // Radio pipe addresses for the 2 nodes to communicate.

void nr_setup(void) {
  pinMode(A0, INPUT);
  radio.begin();
  radio.setAutoAck(0);
  radio.setRetries(0, 1);
  radio.setPayloadSize(NRF_PACKETSIZE);

  if (digitalRead(A0)) {
    Serial.println(F("LEFT RADIO"));
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
  }
  else {
    Serial.println(F("RIGHT RADIO"));
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1, pipes[0]);
  }

  radio.startListening();
  radio.printDetails();
}

void nr_update(void) {

  uint8_t wireless_output_buffer[NRF_PACKETSIZE]; // COUNTER, R , G , B, millis[3],millis[2],millis[1],millis[0]
  wireless_output_buffer[0] = broadcasted_mode;
  wireless_output_buffer[1] = leds_pulse.r;
  wireless_output_buffer[2] = leds_pulse.g;
  wireless_output_buffer[3] = leds_pulse.b;
  uint32_t temp = millis();
  wireless_output_buffer[4] = (uint8_t)(temp >> 0  & 0xFF);
  wireless_output_buffer[5] = (uint8_t)(temp >> 8  & 0xFF);
  wireless_output_buffer[6] = (uint8_t)(temp >> 16 & 0xFF);
  wireless_output_buffer[7] = (uint8_t)(temp >> 24 & 0xFF);

  radio.stopListening();
  if (!radio.write( &wireless_output_buffer, NRF_PACKETSIZE)) {
    Serial.println(F("NRF24 Send Failed."));
  }
  packets_out++;
  radio.startListening();


  while (radio.available()) {
    uint8_t wireless_input_buffer[NRF_PACKETSIZE];
    radio.read( &wireless_input_buffer, NRF_PACKETSIZE);
    wireless_remote_update_time = millis();
    wireless_remote_time = wireless_input_buffer[7];
    wireless_remote_time = wireless_remote_time << 8;
    wireless_remote_time |= wireless_input_buffer[6];
    wireless_remote_time = wireless_remote_time << 8;
    wireless_remote_time |= wireless_input_buffer[5];
    wireless_remote_time = wireless_remote_time << 8;
    wireless_remote_time |= wireless_input_buffer[4];

    wireless_remote_pulse = CRGB(wireless_input_buffer[1],wireless_input_buffer[2],wireless_input_buffer[3]);

    wireless_remote_mode = wireless_input_buffer[0];
    packets_in++;
  }
}
