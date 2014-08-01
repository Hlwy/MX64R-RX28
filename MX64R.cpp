#include "mbed.h"
#include "DebounceIn.h"

//#define COMMUNICATION_DEBUG
//#define DEBUG
//#define LOAD


//INPUTS AND OUTPUTS
DigitalOut RTS(PTA13);// The pin that determines if we are transmitting (logic HIGH) or receiving (logic LOW) on the BOB 10124 UART TO RS485 Converter
Serial motor(PTE0,PTE1);// Serial communication to the motors **gets converted to RS485 by external hardware (BOB 10124) down the line**
Serial pc(USBTX,USBRX);// Serial communication to the pc for debugging
//TESTING APPARATUS CONTROL
DigitalOut red(LED1);//Lights up when hand is closing
DigitalOut green(LED2);//Lights up when hand is opening
DigitalIn open(PTC7);//Switch to open hand
DigitalIn close(PTC3);//Switch to close hand
//MOTOR DEBUGGING
DigitalOut toggle(PTC11);

uint8_t buffer[0x8F];                               //TO STORE ANY COMMANDS NECESSARY
uint8_t parameter[0x8F] = {0};                      // STORES ALL RETURNING PACKET BYTES

//============================================================================================

//============================================================================================

/** Set Torque Enabled
 *
 * @param enableTorque: Can either be 0x00 or 0x01
 *
*/
void SetEnableTorque(uint8_t servoId, uint8_t enableTorque)
{
    //Pre-Established CHECKSUM for motor verification later
    uint8_t checksum_instruction = ~( servoId + 0x04 + 0x03 + 0x18 + enableTorque);     //NOTE: this must match CHECKSUM sent by motor to execute sent commands
    //Flush motor to ensure good data transmission
    while (motor.readable()) {
        motor.getc();
    }

    RTS = 1;                                        //MUST BE HIGH TO SEND PACKETS FROM CONTROLLER TO MOTOR VIA RS485 CONVERTER
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(servoId);                            //MOTOR ID
    motor.putc(0x04);                               //LENGTH
    motor.putc(0x03);                               //REG_WRITE(0x04) COMMAND  [CAN BE CHANGED TO WRITE (0x03)]
    motor.putc(0x18);                               //ENABLE TORQUE REGISTER ADDRESS
    motor.putc(enableTorque);                       //SET ENABLE TORQUE ON(0X01) OR OFF(0X00)
    motor.putc(checksum_instruction);               //CHECKSUM TO BE SENT TO MOTOR TO VERIFY GOOD PACKET TRANSMISSION
    wait_us(400);                                   //Let the hardware catch up to code (200 - 400 works)
    RTS = 0;                                        //MUST BE LOW FOR CONTROLLER TO RECEIVE PACKETS FROM MOTOR

    //Get incoming packets and store them
    while(!(motor.readable())) {};                  //ENSURES WE RECEIVE GOOD BYTES IS IMPLEMENTED THROUGHOUT ENTIRE RETEIVAL PROCESS
    uint8_t header = motor.getc();

    while(!(motor.readable())) {};
    uint8_t header1 = motor.getc();

    while(!(motor.readable())) {};
    uint8_t servoID = motor.getc();

    while(!(motor.readable())) {};
    uint8_t length = motor.getc();

    while(!(motor.readable())) {};
    uint8_t instruction_error_ID = motor.getc();

    //LOOP THROUGH PARAMETERS RECEIVED AND STORE
    if(length > 2) {
        for(uint8_t i = 1; i <= length - 2; i++) {
            while(!(motor.readable())) {};

            uint8_t para = motor.getc();
            parameter[i] = para;
        }
    }
    //Motor's CHECKSUM  ***NOTE: MUST MATCH PRE_ESTABLISHED CHECKSUM FOR PROPER COMMAND EXECUTION
    while(!(motor.readable())) {};
    uint8_t checksum = motor.getc();
    //WAIT FOR HARDWARE TO CATCH UP
    wait_us(170);

#ifdef DEBUG
    pc.printf("Torque Enabled: 0x%02X\r\n", enableTorque);
#endif
}

/** Turn On/Off the LED
 *
 * @param enableLED: can either be 0x00 or 0x01
 *
*/
void SetEnableLED(uint8_t servoId, uint8_t enableLED)
{
    //Pre-Established CHECKSUM for motor verification later
    uint8_t checksum_instruction = ~( servoId + 0x04 + 0x03 + 0x19 + enableLED);     //NOTE: this must match CHECKSUM sent by motor to execute sent commands
    //Flush motor to ensure good data transmission
    while (motor.readable()) {
        motor.getc();
    }

    RTS = 1;                                        //MUST BE HIGH TO SEND PACKETS FROM CONTROLLER TO MOTOR VIA RS485 CONVERTER
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(servoId);                            //MOTOR ID
    motor.putc(0x04);                               //LENGTH
    motor.putc(0x03);                               //REG_WRITE(0x04) COMMAND  [CAN BE CHANGED TO WRITE (0x03)]
    motor.putc(0x19);                               //ENABLE LED REGISTER ADDRESS
    motor.putc(enableLED);                          //TURN LED ON (0X01) OR OFF (0X00)
    motor.putc(checksum_instruction);               //CHECKSUM TO BE SENT TO MOTOR TO VERIFY GOOD PACKET TRANSMISSION
    wait_us(400);                                   //Let the hardware catch up to code (200 - 400 works)
    RTS = 0;                                        //MUST BE LOW FOR CONTROLLER TO RECEIVE PACKETS FROM MOTOR

    //Get incoming packets and store them
    while(!(motor.readable())) {};                  //ENSURES WE RECEIVE GOOD BYTES IS IMPLEMENTED THROUGHOUT ENTIRE RETEIVAL PROCESS
    uint8_t header = motor.getc();

    while(!(motor.readable())) {};
    uint8_t header1 = motor.getc();

    while(!(motor.readable())) {};
    uint8_t servoID = motor.getc();

    while(!(motor.readable())) {};
    uint8_t length = motor.getc();

    while(!(motor.readable())) {};
    uint8_t instruction_error_ID = motor.getc();

    //LOOP THROUGH PARAMETERS RECEIVED AND STORE
    if(length > 2) {
        for(uint8_t i = 1; i != length - 2; i++) {
            while(!(motor.readable())) {};

            uint8_t para = motor.getc();
            parameter[i] = para;
        }
    }

    //Motor's CHECKSUM  ***NOTE: MUST MATCH PRE_ESTABLISHED CHECKSUM FOR PROPER COMMAND EXECUTION
    while(!(motor.readable())) {};
    uint8_t checksum = motor.getc();
    //WAIT FOR HARDWARE TO CATCH UP
    wait_us(170);


#ifdef COMMUNICATION_DEBUG
    //Tell user what was sent to motor
    pc.printf("SENT PACKET: 0xFF 0xFF 0x%02X 0x04 0x04 0x19 0x%02X 0x%02X \r\n", servoId,enableLED,checksum);
    //Send Return Packet to PC
    pc.printf("RETURN PACKET: ");
    pc.printf(" 0x%02X 0x%02X 0x%02X 0x%02X ", header,header1,servoID,length);
    //Loop through the parameter array
    for(int i = 0; i != length - 1; i++) {
        pc.printf("0x%02X ",parameter[i]);
    }
    pc.printf("0x%02X \r\n", checksum);
#endif

#ifdef DEBUG
    //LET USER KNOW WHAT IS BEING MONITORED
    pc.printf("LED Enabled: 0x%02X\r\n", enableLED);
#endif
}

/** Set the goal position of the servo
*
* @param user_goal: Goal Position from 0-0x3FF (MOTOR MAX = 0x3FF)
*/
void SetGoalPosition(uint8_t servoId, uint16_t user_goal)
{
    uint8_t goal_low_byte;
    uint8_t goal_high_byte;
    //BREAK UP SINGLE INPUT INTO TWO BYTES SO MOTOR CAN READ
    goal_low_byte = user_goal;
    goal_high_byte = user_goal >> 8;

    //Pre-Established CHECKSUM for motor verification later
    uint8_t checksum_instruction = ~( servoId + 0x05 + 0x03 + 0x1E + goal_low_byte + goal_high_byte);     //NOTE: this must match CHECKSUM sent by motor to execute sent commands
    //Flush motor to ensure good data transmission
    while (motor.readable()) {
        motor.getc();
    }

    RTS = 1;                                        //MUST BE HIGH TO SEND PACKETS FROM CONTROLLER TO MOTOR VIA RS485 CONVERTER
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(servoId);                            //MOTOR ID
    motor.putc(0x05);                               //LENGTH
    motor.putc(0x03);                               //REG_WRITE(0x04) COMMAND  [CAN BE CHANGED TO WRITE (0x03)]
    motor.putc(0x1E);                               //GOAL POSITION REGISTER ADDRESS
    motor.putc(goal_low_byte);                      //DESIRED GOAL POSITION (LOWEST BYTE)
    motor.putc(goal_high_byte);                     //DESIRED GOAL POSITION (HIGHEST BYTE)
    motor.putc(checksum_instruction);               //CHECKSUM TO BE SENT TO MOTOR TO VERIFY GOOD PACKET TRANSMISSION
    wait_us(400);                                   //Let the hardware catch up to code (200 - 400 works)
    RTS = 0;                                        //MUST BE LOW FOR CONTROLLER TO RECEIVE PACKETS FROM MOTOR

    //Get incoming packets and store them
    while(!(motor.readable())) {};                  //ENSURES WE RECEIVE GOOD BYTES IS IMPLEMENTED THROUGHOUT ENTIRE RETEIVAL PROCESS
    uint8_t header = motor.getc();

    while(!(motor.readable())) {};
    uint8_t header1 = motor.getc();

    while(!(motor.readable())) {};
    uint8_t servoID = motor.getc();

    while(!(motor.readable())) {};
    uint8_t length = motor.getc();

    while(!(motor.readable())) {};
    uint8_t instruction_error_ID = motor.getc();

    //LOOP THROUGH PARAMETERS RECEIVED AND STORE
    if(length > 2) {
        for(uint8_t i = 1; i != length - 2; i++) {
            while(!(motor.readable())) {};

            uint8_t para = motor.getc();
            parameter[i] = para;
        }
    }
    //Motor's CHECKSUM  ***NOTE: MUST MATCH PRE_ESTABLISHED CHECKSUM FOR PROPER COMMAND EXECUTION
    while(!(motor.readable())) {};
    uint8_t checksum = motor.getc();
    //WAIT FOR HARDWARE TO CATCH UP
    wait_us(170);

#ifdef DEBUG
    uint16_t goal = (((uint16_t)goal_high_byte) << 8) | ((uint16_t)goal_low_byte);
    pc.printf("Set Goal Position: %hu \r\n", goal);//MAKE EASIER TO READ POSSIBLY CONVERTING TO ACTUAL SPEED WITH MULTIPLIER???
#endif

}


/** Set the moving speed of servo towards goal position
*
* @param user_speed: Speed of servo to goal position (CAN BE 0 - 0x3FF MOTOR MAX)
*/
void SetMovingSpeed(uint8_t servoId, uint16_t user_speed)
{
    uint8_t speed_low_byte;
    uint8_t speed_high_byte;
    //BREAK UP SINGLE INPUT INTO TWO BYTES SO MOTOR CAN READ
    speed_low_byte = user_speed;
    speed_high_byte = user_speed >> 8;

    //Pre-Established CHECKSUM for motor verification later
    uint8_t checksum_instruction = ~( servoId + 0x05 + 0x03 + 0x20 + speed_low_byte + speed_high_byte);     //NOTE: this must match CHECKSUM sent by motor to execute sent commands
    //Flush motor to ensure good data transmission
    while (motor.readable()) {
        motor.getc();
    }

    RTS = 1;                                        //MUST BE HIGH TO SEND PACKETS FROM CONTROLLER TO MOTOR VIA RS485 CONVERTER
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(servoId);                            //MOTOR ID
    motor.putc(0x05);                               //LENGTH
    motor.putc(0x03);                               //REG_WRITE(0x04) COMMAND  [CAN BE CHANGED TO WRITE (0x03)]
    motor.putc(0x20);                               //MOVING SPEED REGISTER ADDRESS
    motor.putc(speed_low_byte);                     //DESIRED MOVING SPEED (LSB)
    motor.putc(speed_high_byte);                    //DESIRED MOVING SPEED (MSB)
    motor.putc(checksum_instruction);               //CHECKSUM TO BE SENT TO MOTOR TO VERIFY GOOD PACKET TRANSMISSION
    wait_us(400);                                   //Let the hardware catch up to code (200 - 400 works)
    RTS = 0;                                        //MUST BE LOW FOR CONTROLLER TO RECEIVE PACKETS FROM MOTOR

#ifdef REGISTER
    //Get incoming packets and store them
    while(!(motor.readable())) {};                  //ENSURES WE RECEIVE GOOD BYTES IS IMPLEMENTED THROUGHOUT ENTIRE RETEIVAL PROCESS
    uint8_t header = motor.getc();

    while(!(motor.readable())) {};
    uint8_t header1 = motor.getc();

    while(!(motor.readable())) {};
    uint8_t servoID = motor.getc();

    while(!(motor.readable())) {};
    uint8_t length = motor.getc();

    while(!(motor.readable())) {};
    uint8_t instruction_error_ID = motor.getc();

    //LOOP THROUGH PARAMETERS RECEIVED AND STORE
    if(length > 2) {
        for(uint8_t i = 1; i != length - 2; i++) {
            while(!(motor.readable())) {};

            uint8_t para = motor.getc();
            parameter[i] = para;
        }
    }
    //Motor's CHECKSUM  ***NOTE: MUST MATCH PRE_ESTABLISHED CHECKSUM FOR PROPER COMMAND EXECUTION
    while(!(motor.readable())) {};
    uint8_t checksum = motor.getc();
    //WAIT FOR HARDWARE TO CATCH UP
    wait_us(170);
#endif

#ifdef DEBUG
    uint16_t speed = (((uint16_t)speed_high_byte) << 8) | ((uint16_t)speed_low_byte);
    pc.printf("Set Moving Speed: %hu \r\n", speed);//MAKE EASIER TO READ POSSIBLY CONVERTING TO ACTUAL SPEED WITH MULTIPLIER???
#endif
}

void Reset(uint8_t servoId)
{
    //Pre-Established CHECKSUM for motor verification later
    uint8_t checksum_instruction = ~( servoId + 0x02 + 0x06);     //NOTE: this must match CHECKSUM sent by motor to execute sent commands
    //Flush motor to ensure good data transmission
    while (motor.readable()) {
        motor.getc();
    }

    RTS = 1;                                        //MUST BE HIGH TO SEND PACKETS FROM CONTROLLER TO MOTOR VIA RS485 CONVERTER
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(servoId);                            //MOTOR ID
    motor.putc(0x02);                               //LENGTH
    motor.putc(0x06);                               //COMMAND TO RESET
    motor.putc(checksum_instruction);               //CHECKSUM TO BE SENT TO MOTOR TO VERIFY GOOD PACKET TRANSMISSION
    wait_us(400);                                   //Let the hardware catch up to code (200 - 400 works)
    RTS = 0;                                        //MUST BE LOW FOR CONTROLLER TO RECEIVE PACKETS FROM MOTOR

    //Get incoming packets and store them
    while(!(motor.readable())) {};                  //ENSURES WE RECEIVE GOOD BYTES IS IMPLEMENTED THROUGHOUT ENTIRE RETEIVAL PROCESS
    uint8_t header = motor.getc();

    while(!(motor.readable())) {};
    uint8_t header1 = motor.getc();

    while(!(motor.readable())) {};
    uint8_t servoID = motor.getc();

    while(!(motor.readable())) {};
    uint8_t length = motor.getc();

    while(!(motor.readable())) {};
    uint8_t instruction_error_ID = motor.getc();

    //LOOP THROUGH PARAMETERS RECEIVED AND STORE
    if(length > 2) {
        for(uint8_t i = 1; i != length - 2; i++) {
            while(!(motor.readable())) {};

            uint8_t para = motor.getc();
            parameter[i] = para;
        }
    }
    //Motor's CHECKSUM  ***NOTE: MUST MATCH PRE_ESTABLISHED CHECKSUM FOR PROPER COMMAND EXECUTION
    while(!(motor.readable())) {};
    uint8_t checksum = motor.getc();
    //WAIT FOR HARDWARE TO CATCH UP
    wait_us(170);

#ifdef DEBUG
    //LET USER KNOW WHAT IS BEING MONITORED
    pc.printf("Reset Successful");
#endif
}

void Action(void)
{
    //Pre-Established CHECKSUM for motor verification later
    uint8_t checksum = ~( 0xFE + 0x02 + 0x05) ;     //NOTE: this must match CHECKSUM sent by motor to execute sent commands
    //Flush motor to ensure good data transmission
    while (motor.readable()) {
        motor.getc();
    }

    RTS = 1;                                        //MUST BE HIGH TO SEND PACKETS FROM CONTROLLER TO MOTOR VIA RS485 CONVERTER
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(0xFF);                               //HEADER BYTE
    motor.putc(0xFE);                               //BROADCAST ID
    motor.putc(0x02);                               //LENGTH
    motor.putc(0x05);                               //ACTION COMMAND
    motor.putc(checksum);                           //CHECKSUM TO BE SENT TO MOTOR TO VERIFY GOOD PACKET TRANSMISSION
    wait_us(400);                                   //Let the hardware catch up to code (200 - 400 works)
    RTS = 0;                                        //MUST BE LOW FOR CONTROLLER TO RECEIVE PACKETS FROM MOTOR
    wait_ms(20);                                    //WAIT TO PREVENT POSSIBLE OVERFLOWS IN CONTROLLER
#ifdef DEBUG
    pc.printf("Action\r\n");
#endif
}

//===============================================================
//                  MAIN PROGRAM
//===============================================================
int main()
{
    motor.baud(57600);
    pc.baud(57600);

    red = 1; //TURNS OFF THE LED       
    green = 1;//TURNS OFF THE LED

    int i = 0;//Variable to count the goal position 
    int t = 5; //Input a fixed time to wait in between command packets to ensure no data clobbered

    while(1) {
        //If switch is pushed to open
        if(open == 0) {
            //Check current position is at the origin(POSSIBILITY OF BETTER CHECKING SYSTEM LATER)
            if(i != 0) {
                i -= 50;
                green = 0;
                SetGoalPosition(0x01,i);
                wait_ms(t);
                SetMovingSpeed(0x01,1023);
                wait_ms(t);
                green = 1;
            }
        }
        //If switch is pushed to close
        if(close == 0) {
            //Check current position is at the max(POSSIBILITY OF BETTER CHECKING SYSTEM LATER)
            if(i != 3500) { //this can change depending on physical systems limitations (BE CAREFEUL NOT TO HAVE TO HIGH)
                i += 50;
                red = 0;
                SetGoalPosition(0x01,i);
                wait_ms(t);
                SetMovingSpeed(0x01,1023);
                wait_ms(t);
                red = 1;
            }
        }
    }
}