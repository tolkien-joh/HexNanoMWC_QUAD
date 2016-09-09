#ifndef _DEBUG_H
#define _DEBUG_H

static int delay_cnt = 0;

void dump_motor_parameter() {
     Serial.print("m ");
     Serial.print(motor[0]);
     Serial.print(".");
     uint16_t i2 = ((motor[0]<<4) - 16000) + 128;
     Serial.print(i2 & 0xffff, HEX);
     Serial.print(",");
     Serial.print(motor[1]);
     Serial.print(".");
     i2 = ((motor[1]<<4) - 16000) + 128;
     Serial.print(i2 & 0xffff, HEX);
     Serial.print(",");
     Serial.print(motor[2]);
     Serial.print(".");
     i2 = 2047-(((motor[2]-1000)<<1)+16)>>8;
     Serial.print(i2 & 0xf, HEX);
     Serial.print(".");
     i2 = 2047-(((motor[2]-1000)<<1)+16)&0xFF;
     Serial.print(i2 & 0xff, HEX);
     Serial.print(",");
     Serial.print(motor[3]);
     Serial.print(".");
     i2 = (((motor[3]-1000)<<1)+16)>>8;
     Serial.print(i2 & 0xf, HEX);
     Serial.print(".");
     i2 = (((motor[3]-1000)<<1)+16)&0xFF;
     Serial.print(i2 & 0xff, HEX);
     Serial.print(",");
     Serial.println(".");
}

void dump_controller_parameter() {
     Serial.print(rcData[3]);
     Serial.print(", ");
     Serial.print(rcCommand[THROTTLE]);
     Serial.print(", ");
     Serial.print(axisPID[ROLL]);
     Serial.print(", ");
     Serial.print(axisPID[PITCH]);
     Serial.print(", ");
     Serial.println(axisPID[YAW]);
}

void print_debug_msg() {
     delay_cnt++;
     if (delay_cnt > 50) {

       dump_controller_parameter();
       dump_motor_parameter();

       delay_cnt = 0;
     }
}

#endif	/* _DEBUG_H */
