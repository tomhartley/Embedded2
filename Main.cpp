#include "mbed.h"
#include "rtos.h"
 
//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12
 
//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  
 
//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20
 
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
 
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
 
//Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards

// speed
int8_t orState = 0;    //Rotot offset at motor state 0
volatile float speed = 0;
volatile float max_spd = 0;

// rotations
volatile uint16_t rotations = 0;
volatile uint16_t max_rotations = 0;

// direction
volatile int8_t direction = -1;
volatile uint8_t dir_prev;

// speed times
volatile int avg_time = 0; // 1/6 of a cycle
volatile int prev_times[] = {0,0,0,0,0,0};
Timer rot;

// mutex
Mutex mut;

// set mode
volatile int dist = 0; //0 == set velocity; 1 == set distance


Serial pc(SERIAL_TX, SERIAL_RX);

// Threads
Thread spinThread;
Thread cycleThread;
 
//Status LED
DigitalOut led1(LED1);
 
//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);

 
//Set a given drive state
void motorOut(int8_t driveState, float scale){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.write(0.0);
    if (~driveOut & 0x02) L1H.write(scale);
    if (~driveOut & 0x04) L2L.write(0.0);
    if (~driveOut & 0x08) L2H.write(scale);
    if (~driveOut & 0x10) L3L.write(0.0);
    if (~driveOut & 0x20) L3H.write(scale);
    
    //Then turn on
    if (driveOut & 0x01) L1L.write(scale);
    if (driveOut & 0x02) L1H.write(0.0);
    if (driveOut & 0x04) L2L.write(scale);
    if (driveOut & 0x08) L2H.write(0.0);
    if (driveOut & 0x10) L3L.write(scale);
    if (driveOut & 0x20) L3H.write(0.0);
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }
 
//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 1.0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}


void photoInterrupt_1(){
    rot.stop();
    avg_time = rot.read_us()*3;
    rot.reset();
    rot.start();
    rotations+=1;
    
    for (int i = 1; i<6; i++) {
        prev_times [i-1] = prev_times [i];
    }
    prev_times[5] = avg_time;
    if (dir_prev == (2)) {
        direction = -1;
    } else {
        direction = 1;
    }

    dir_prev = 0;
}

void photoInterrupt_2(){
    rot.stop();
    avg_time = rot.read_us()*3;
    rot.reset();
    rot.start();
    rotations+=1;
    
    for (int i = 1; i<6; i++) {
        prev_times [i-1] = prev_times [i];
    }
    prev_times[5] = avg_time;

    if (dir_prev == (0)) {
        direction = -1;
    } else {
        direction = 1;
    }

    dir_prev = 1;
}

void photoInterrupt_3(){
    rot.stop();
    avg_time = rot.read_us()*3;
    rot.reset();
    rot.start();
    rotations+=1;
    
    for (int i = 1; i<6; i++) {
        prev_times [i-1] = prev_times [i];
    }
    prev_times[5] = avg_time;

    if (dir_prev == (1)) {
        direction = -1;
    } else {
        direction = 1;
    }

    dir_prev = 2;
}

inline int16_t note_freq(char note){
    switch (note) { 
        case 'c' : return (4186);
        case 'd' : return (4699);
        case 'e' : return (5274);
        case 'f' : return (5588);
        case 'g' : return (6272);
        case 'a' : return (7040);
        case 'b' : return (7902);
        default : return 7902;
    }
}

// set speed and direction of rotor
inline void setPWMvelocity(float spd) {
    mut.lock();
    if (spd<0) {
        speed = -1.0*spd;
        lead = -2;
    } else {
        speed = spd;
        lead = 2;
    }
    if (speed>1) {
        speed = 1;
    } else if (speed<0) {
        speed = 0;
    }
    mut.unlock();
}

inline float getPWMvelocity(){
    mut.lock();
    float ret = speed*(lead/2);
    mut.unlock();
    return ret;
}

// gets speed as positive or negative depending on direction
inline float velocity(float in) {
    mut.lock();
    float ret = in*direction;
    mut.unlock();
    return ret;
}

void spinMotor() {
    int8_t intState = 0;
    float e_t = 0, e_t_prev = 0, de_dt = 0;
    float kp = 0.013, kd = 0.000742;
    Timer t;
    t.start();
    while(1) {
        if (t.read_ms() > 30){
            t.stop();
            int diff_time = t.read_ms();
            t.reset();
            e_t_prev = e_t;
            e_t = max_spd - velocity(1000000.0/avg_time);
            de_dt = (e_t - e_t_prev)*1000.0/diff_time;
            float speed_adjust = kp*e_t + kd*de_dt;
            setPWMvelocity(getPWMvelocity()+speed_adjust);
            t.start();
        }
        intState = readRotorState();
        motorOut((intState-orState+lead+6)%6, speed); //+6 to make sure the remainder is positive
    }
}

int split_notes(char regex[49]) {
    int i = 1;
    while(i <= 49) {
        int sf = 0;
        switch (regex[i+1]) {
            case '#' : sf = 1;
            case '^' : sf = 2;
        }
        switch (regex [i]) { 
            case 'C' : 
            case 'D' : 
            case 'E' : 
            case 'F' : 
            case 'G' : 
            case 'A' : 
            case 'B' : 
            default : 
        }
    }

}

void rot_commands(char* regex) {
    bool Rexists = false;
    bool Vexists = false;
    bool foundend = false;
    int RNumstart = 0;
    int RNumstop = 0;
    int VNumstart = 0;
    int VNumstop = 0;
    char* Rnumstr = new char;
    char* Vnumstr = new char;
    float Rnum = 0;
    float Vnum = 0;
    if (regex[0] == 'R') {
        Rexists = true;
        RNumstart = 1;
        for (int i = 0; i <= 48; i++) {
            if(regex[i] == 'V') {
                Vexists = true;
                RNumstop = i-1;
                VNumstart = i+1;
            } else if(regex[i] == ';' && !foundend ) {
                if(Vexists) {
                    VNumstop = i-2;
                } else {
                    RNumstop = i-2;
                }
                foundend = true;
            }
        }
    } else if (regex[0] == 'V') {
        Rexists = false;
        Vexists = true;
        VNumstart = 1;
        for (int i = 0; i <= 48; i++) {
            if(regex[i] == ';' && !foundend ) {
                if(Vexists) {
                    VNumstop = i-2;
                } else {
                    RNumstop = i-2;
                }
                foundend = true;
            }
        }
    }
    if(Rexists) {
        for (int i = RNumstart; i <= RNumstop; i++) {
            Rnumstr[i-RNumstart]=regex[i];
        }
        Rnum = atof(Rnumstr);
    } 
    if(Vexists) {
        for (int i = VNumstart; i <= VNumstop; i++) {
            Vnumstr[i-VNumstart]=regex[i];
        }
        Vnum = atof(Vnumstr);
    }
    if(Rexists && Vexists && (Vnum < 0)) {
        Vnum = -Vnum;
    }
    
    // set the global variables.
    if (Rexists && !Vexists) {
        mut.lock();
        max_rotations = Rnum;
        dist = 1;
        mut.unlock();
    }
    else if (!Rexists && Vexists) {
        mut.lock();
        max_spd = Vnum;
        dist = 0;
        mut.unlock();
    }
    // else if ()
}

//Main
int main() {

    
    static char buffer[49];
    for (int i = 0; i <= 48; i++) { 
        buffer[i] = ';';
    }
    static uint8_t count = 0;
    
    static int16_t freq = 4186;
    
    I1.rise(photoInterrupt_1);
    I2.rise(photoInterrupt_2);
    I3.rise(photoInterrupt_3);
    
    L1L.period_us(1000000/freq);
    L1H.period_us(1000000/freq);
    L2L.period_us(1000000/freq);
    L2H.period_us(1000000/freq);
    L3L.period_us(1000000/freq);
    L3H.period_us(1000000/freq);
    
    //Initialise the serial port
    
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    rot.start();
    spinThread.start(spinMotor);
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {       
        char c;    
        if (pc.readable()){
            c = pc.getc();
            pc.putc(c);
            buffer[count] = c;
            count++;
            if (count > 47 || c == '\r' || c == '\n'){
                pc.printf("\r\n");
                count = 0;

                if(buffer[0] == 'T'){
                    freq = note_freq(buffer[1]);

                    int test = split_notes(buffer);
                    pc.printf("Test = '%i'\r\n", test);
                    pc.printf("New frequency = '%f'\r\n", freq);
                    L1L.period(1/freq);
                    L1H.period(1/freq);
                    L2L.period(1/freq);
                    L2H.period(1/freq);
                    L3L.period(1/freq);
                    L3H.period(1/freq);
                }
                else if (buffer[0] == 'R' || buffer[0] == 'V'){
                    rot_commands(buffer);
                }
                else if (buffer[0] == 's'){
                    mut.lock();
                    pc.printf("desired speed = %f, speed = %f, direction = %d", max_spd, velocity(1000000.0/avg_time), direction);
                    mut.unlock();
                }
                else {
                    count = 0;
                }
                
                for (int i = 0; i <= 48; i++) { 
                    buffer[i] = ';';
                }
                
            }
        }
    }
}
