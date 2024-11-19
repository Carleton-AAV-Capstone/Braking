#define TX_PIN 17
#define RX_PIN 16

int readByte()
{
  char c;

  if(Serial2.readBytes(&c, 1) == 0) 
  {
    return -1;  // Return -1 if no byte received
  }
  return (byte)c;  // Return the byte read
}

// Function to exit safe mode and enable motor movement
// This must be called when the controller restarts or after any error
void exitSafeStart()
{
  Serial2.write(0x83);  // Command to exit safe start
}

// Function to set motor speed
// Speed should be in the range -3200 to 3200
void setMotorSpeed(int speed)
{
  // Handle negative speed (reverse)
  if (speed < 0)
  {
    Serial2.write(0x86);  // Motor reverse command
    speed = -speed;       // Convert speed to positive
  }
  else
  {
    Serial2.write(0x85);  // Motor forward command
  }

  // Send the speed (two bytes, split between 5 and 7 bits)
  Serial2.write(speed & 0x1F);      // Send the least significant 5 bits
  Serial2.write((speed >> 5) & 0x7F);  // Send the most significant 7 bits
}

// Function to get a scaled sensor value from command A1
float getA1_scaled()
{
  Serial2.write(0xA1);  // Command A1
  Serial2.write(14);    // Send the scaling factor
  return ((float)(readByte() + 256 * readByte())) / 32.0;  // Scale and return
}

void motor_controller_setup(){
  //  Initialize Serial2 with a baud rate of 9600 bps
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Initialize regular serial communication for debugging
  Serial.begin(115200);

  // Delay to allow motor controller to initialize (5 ms minimum)
  delay(5);
  
  // Send the baud rate auto-detect byte (0xAA) to the motor controller
  Serial2.write(0xAA);

  // Exit safe start to enable the motor
  exitSafeStart();
}