Communication with the Ø32 BLDC controller requires using a RS485 transceiver like [MAX485](https://www.amazon.com/Transceiver-Instrument-Communication-Development-Accessories/dp/B094MKRTRC) or [THVD1420DRLR](https://www.digikey.com/en/products/detail/texas-instruments/THVD1420DRLR/15856992). Use 2-pin JST-SH cables to connect the transceiver A and B pins to the Ø32. Connect a separate microcontroller's UART to the transceiver to send RS485 commands.

Communication protocol:
  1. transmit command byte (control modes, whether you are requesting data)
  2. transmit command data byte (target speed, or what data to request)
  3. receive n bytes of data response (determined by firmware)

Example in Arduino IDE:
```
#define CMD_SET_VOLTAGE (0b10000000)
#define CMD_SET_SPEED (0b10010000)
#define CMD_SET_POSITION (0b10100000)
#define CMD_SET_CURRENT (0b10110000)
#define CMD_GET_POSITION (0b11000000)
#define CMD_GET_SPEED (0b11010000)
#define CMD_GET_CURRENT (0b11100000)
#define CMD_GET_TEMPERATURE (0b11110000)

#define RS485_DE 8 //Driver enable for the transciever, connect to a GPIO

uint8_t uart2_RX[10] = {0}; //response from Ø32

uint8_t motor_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx){
  while(Serial2.available()) Serial2.read(); //clear rx buffer

  uint8_t uart2_TX[3] = {0}; //command RS485 to Ø32
  uart2_TX[0] = CMD_TYPE | addr;
  uart2_TX[1] = (data >> 7) & 0b01111111;
  uart2_TX[2] = (data)      & 0b01111111;

  digitalWrite(RS485_DE, HIGH);
  Serial2.write(uart2_TX, 3);
  Serial2.flush();
  digitalWrite(RS485_DE, LOW);
  delay(1); //delay can be reduced
  int numread = Serial2.readBytes(rx, 10);
  return numread; 
}

void setup() { 
  Serial.begin(115200); //UART for printing
  Serial2.begin(115200); // UART for RS485 output (RX2=pin7, TX2=pin8)
  Serial2.setTimeout(1);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
}

void loop() {
  // send voltage command to motor ID 6 
  motor_cmd(6, CMD_SET_VOLTAGE, 100, uart2_RX);
  delay(1);

  for(int i = 0; i< sizeof(uart2_RX); i++){
    Serial.println(uart2_RX[i]);
  }
}
```

Hardware here: https://github.com/qwertpas/O32controller
