//
// Description: LIRIC IR imager front-end controller
// Platform:    Arduino Mega 2560 plus V3 motor shield
// Alias:       Super Nuj-Master 9000+
// Author:      ASP
// Date:        2024-07-11
//
#include <OneWire.h>
#include <DallasTemperature.h>

#define Version "LIRIC Controller V1.03"

// Pin allocation
#define FAN_RPM   20     // Read heat sink fan speed. Must be IRQ pin
#define PUMP_RPM  21     // Read pump speed. Must be IRQ pin
#define PUMP_PWM  6      // Sets pump speed
#define PUMP_FREQ 25000  // PWM frequency (25kHz)

// One-wire Dallas ds18xxx temperature sensor
#define TEMP_PIN 5       // Dallas 1-wire bus (orange socket)
#define TEMP_MAX 3       // Maximum 1-wire devices
OneWire           temp_ow(TEMP_PIN);
DallasTemperature temp( &temp_ow );

                         // Index into look-up table
#define REVERSE  LOW     // =0 Target if moving counter-clockwise
#define FORWARD  HIGH    // =1 Target if moving clockwise
#define TARGET   2       // =2 Target final position after nudges

#define NUDGES   10      // Maximum nudges allowed
#define EXCESS    8      // Excessive nudges, switch to MOVE_TOL
#define PLACES    5      // Known placement positions
#define WINDOW    10     // Just use nudges if this close to target 
#define SETTLE    150UL  // [ms] Nudge settling time
#define MAXRUN    10000UL// [ms] Maximum motor run-time
#define TIMEOUT  -1      // Timeout error 
#define LEARNS    3      // Max learn coarse attempts
#define MOVE_TOL  4      // Coarse motion tolerance
#define FINE_TOL  2      // Nudge motion tolerance
#define SCALE     3      // Nudge scaling factor
#define DECEL     400UL  // [ms] Deceleration pause

typedef struct axis_s {
  int ROTAT;  // I/O pins
  int SPEED;
  int BRAKE;
  int POWER;
  int ANGLE;

  char axis;  // Axis name
  int cmd;    // Command
  int index;  // Table index
  int speed;  // PWM speed
  int rotat;  // Rotation direction
  int tol;    // Tolerance
  int angle;  // Actual position
  int coarse; // Coarse position
  int overun; // Coarse error
  int target; // Target position
  int error;  // Target error
  int nudges; // Nudge count
  int nudge;  // [ms] Nudge length
  int timed;  // [ms] Time to target
  unsigned long millis; // [ms] Used for timing
  unsigned long maxrun; // [ms] Max motor run-time
  unsigned long settle; // [ms] Settling time
  int table[TARGET+1][PLACES];
} axis_t;

axis_t vert = {12,3,9,A0,A3, // Pins: rotat, speed, brake, power, angle
               'y','c',2,255,FORWARD,MOVE_TOL,0,0,0,0,0,0,0,0,0UL,0UL,0UL,
//               +6mm  +2mm  0  -2mm -6mm     // Offset
//                 a     b   c    d    e      // Y-axis position name
               {{ 920, 270, 707, 141,  495 }, // Reverse corrected
                { 909, 257, 697, 130,  484 }, // Forward corrected
                { 913, 262, 702, 136,  490 }} // Target position
}; 

axis_t horz = {13,11,8,A1,A2, // Pins: rotat, speed, brake, power, angle
               'X','C',2,255,FORWARD,MOVE_TOL,0,0,0,0,0,0,0,0,0UL,0UL,0UL,
//                 A    B    C     D    E      // X-axis position name
               {{ 954, 305, 734,  177, 525 },  // Reverse corrected
                { 940, 290, 721,  165, 515 },  // Forward corrected
                { 950, 300, 729,  171, 520 }}  // Target position
};

int fan_count  = 0; // Pump fan RPM
int pump_count = 0; // Pump motor RPM 
int pump_pwm = 128; // Default=50%. PWM duty cycle [0-255]
bool test = false;  // Experimental test mode
String Str = "";    // Dummy string for concatenation

void setup() {
  Serial.begin(19200);

  // Use timer #4 pin6 to set PWM freq.=25kHz and default duty cycle
  // First clear timer control registers
  TCCR4A = TCCR4B = 0;

  // PMW frequency set via ICR4, duty cycle set via OCR4A
  ICR4  = F_CPU/PUMP_FREQ/2 - 1;      // Set base frequency
  OCR4A = ICR4 * (pump_pwm / 255.0f); // Set duty cycle

  // Toggle A=pin6. B=pin7 & C=pin8 operate as normal. 
  // Mode 8 (Phase corr. PWMW, TOP=ICR4). System clock unscaled 
  TCCR4A = _BV(COM4A1) | _BV(WGM41);
  TCCR4B = _BV(WGM43)  | _BV(CS40); 

  // For 1-wire Dallas temperature sensor
  temp.begin();
  temp.setResolution(12); // Resolution [9-12] bits)

  // Set pin modes. Others use default
  pinMode(vert.ROTAT, OUTPUT);
  pinMode(vert.SPEED, OUTPUT);
  pinMode(vert.BRAKE, OUTPUT);
  pinMode(horz.ROTAT, OUTPUT);
  pinMode(horz.SPEED, OUTPUT);
  pinMode(horz.BRAKE, OUTPUT);

  pinMode( PUMP_PWM, OUTPUT       );
  pinMode( PUMP_RPM, INPUT_PULLUP ); 
  pinMode( FAN_RPM,  INPUT_PULLUP ); 

  // Set default speed
  analogWrite(vert.SPEED, vert.speed);
  analogWrite(horz.SPEED, horz.speed);
//  analogWrite(PUMP_PWM, pump_pwm  ); // DOES NOT WORK WITH 25KHz PWM WATER PUMP

  // Set default start positions
  vert.target = vert.table[TARGET][vert.index];
  if (reached(vert.target, vert.angle=analogRead(vert.ANGLE), vert.tol, &vert.rotat, &vert.error)) {
    vert.coarse = vert.target = vert.nudges = vert.timed = 0; // Already there
  } else { // Initiate move
    digitalWrite(vert.ROTAT, vert.rotat);
    vert.coarse = vert.table[vert.rotat][vert.index];
    vert.millis = millis();             // Mark start time
    vert.maxrun = vert.millis + MAXRUN; // Calc max run-time
  }

  horz.target = horz.table[TARGET][horz.index];
  if (reached(horz.target, horz.angle=analogRead(horz.ANGLE), horz.tol, &horz.rotat, &horz.error )) {
    horz.coarse = horz.target = horz.nudges = horz.timed = 0; // Already there
  } else { // Initiate move
    digitalWrite(horz.ROTAT, horz.rotat);
    horz.coarse = horz.table[horz.rotat][horz.index];
    horz.millis = millis();             // Mark start time
    horz.maxrun = horz.millis + MAXRUN; // Calc max run-time
  }
}

void loop() {
  char   cmd;
  axis_t *a;

  if (Serial.available()) { 
    // Select axis
    if (isupper(cmd = Serial.read()))
      a=&horz;
    else
      a=&vert;
    
    // Commands
    switch ( toupper(cmd) ) {
      case 'S': // Halt
        a->target = a->coarse = 0;    // Cancel request
        digitalWrite(a->BRAKE, HIGH); // Brake ON
        Serial.println(Str+cmd+' '+a->angle);
        break;

      case 'A': // +6mm
      case 'B': // +2mm
      case 'C': //  Centre
      case 'D': // -2mm
      case 'E': // -6mm
        a->cmd   = cmd;
        a->index = cmd - (isupper(cmd)?'A':'a');
        a->target = a->table[TARGET][a->index];
        a->tol = FINE_TOL;
        if (reached(a->target, (a->angle=analogRead(a->ANGLE)), a->tol, &a->rotat, &a->error)) {
          Serial.println(Str+char(cmd)+' '+a->angle+' '+a->error+' '+a->nudges+' '+a->timed );
          a->coarse = a->target = a->nudges = a->timed = 0; // Already there
        } else { // Initiate move
           digitalWrite(a->ROTAT, a->rotat);
          if ( abs(a->error) > WINDOW )
            a->coarse = update(&a->table[a->rotat][a->index], 0, 1023 );
          a->millis = millis();           // Mark start time
          a->maxrun = a->millis + MAXRUN; // Calc max run-time
          Serial.println(Str+char(cmd)+' '+a->angle +' '+a->error+' '+a->nudges+' '+a->timed );
        }
        break;

      case 'G': // Go continuous
        a->coarse = -1023;             // Go continuously ...
        update(&a->coarse, 0, 1023);   // ... unless limited to supplied value
        a->millis = millis();          // For timing
        a->maxrun = millis() + MAXRUN; // ... up to maximum run-time
        Serial.println(Str+char(cmd)+' '+(a->angle=analogRead(a->ANGLE)));
        break;

      case 'W': // Where am I?
        if (a->target) { // Move in progress
          reached(a->target, (a->angle=analogRead(a->ANGLE)), a->tol, NULL, &a->error);
          Serial.println(Str+char(cmd)+' '+a->angle+' '+a->error+' '+a->nudges+' '+(millis()-a->millis));            
        } else { // Move completed
          if (reached( a->table[TARGET][a->index], (a->angle=analogRead(a->ANGLE)), MOVE_TOL, NULL, &a->error) )
            Serial.println(Str+char(a->cmd)+' '+a->angle+' '+a->error+' '+a->nudges+' '+a->timed);
          else // Out of tolerance
            Serial.println(Str+char(cmd)+' '+a->angle+' '+a->error+' '+a->nudges+' '+a->timed);
          a->nudges = a->timed = 0; // Zero if you ask again
        }
        break;

      case 'N': // Nudge
        update(&a->nudge, 1, 1000);
        digitalWrite(a->BRAKE, LOW ); // Brakes OFF
        delay(a->nudge);
        digitalWrite(a->BRAKE, HIGH); // Brakes ON
        delay(SETTLE);
        Serial.println(Str+cmd+' '+(a->angle=analogRead(a->ANGLE))+' '+a->nudge);
        break;

      case 'M': // Motor milliamps [mA]
        Serial.println(Str+char(cmd)+' '+((unsigned long)analogRead(a->POWER)*100000UL)/33769UL);
        break;

      case 'F': // Forward motor
        digitalWrite(a->ROTAT, (a->rotat = FORWARD)); // Clockwise
        Serial.println(Str+char(cmd)+' '+(a->angle=analogRead(a->ANGLE)));
        break;

      case 'R': // Reverse motor
        digitalWrite(a->ROTAT, (a->rotat = REVERSE)); // Counter-Clockwise
        Serial.println(Str+char(cmd)+' '+(a->angle=analogRead(a->ANGLE)));
        break;

      case 'V': // Get/set velocity
        analogWrite(a->SPEED, update(&a->speed, 0, 255));
        Serial.println(Str+char(cmd)+' '+a->speed);
        break;

      case 'T': { // Get internal & external temperatures
        temp.requestTemperatures();
        if ( TEMP_MAX == 3)
          Serial.println(Str+char(cmd)+' '+temp.getTempCByIndex(0)+' '+temp.getTempCByIndex(1)+' '+temp.getTempCByIndex(2));
        else
          Serial.println(Str+char(cmd)+' '+temp.getTempCByIndex(0)+' '+temp.getTempCByIndex(1));
      } break;

      case 'P': // Set/Get pump speed - BLOCKING
      //Conventional PWM modulation - DOES NOT WORK WITH PC PUMP
      //analogWrite( PUMP_PWM, update( &pump_speed, 0, 255 ) );

        // Set duty cycle in timer Output Control Register 4A
        OCR4A = ICR4 * ( update( &pump_pwm, 0, 255 ) / 255.0f );

        fan_count = pump_count = 0;
        attachInterrupt(digitalPinToInterrupt(FAN_RPM ), pump_isr, RISING );
        attachInterrupt(digitalPinToInterrupt(PUMP_RPM), fan_isr,  RISING );
        delay( 1000 ); // Wait a second
        detachInterrupt(digitalPinToInterrupt(FAN_RPM ));
        detachInterrupt(digitalPinToInterrupt(PUMP_RPM));
        Serial.println(Str+char(cmd)+' '+pump_pwm+' '+pump_count+' '+fan_count);
      break;

      case '?': // Help
        Serial.println(Str+"Cmd UPCASE=X-AXIS, locase=y-axis, []=Set value");
        Serial.println("S,s(top)");
        Serial.println("G,g(o to [0-1023])");
        Serial.println("F,f(orward)");
        Serial.println("R,r(everse)");
        Serial.println("W,w(here)");
        Serial.println("M,m(otor mA)");
        Serial.println("V,v(elocity [0-255])");
        Serial.println("N,n(udge [1-1000])");
        Serial.println("T,t(emperatures C) T0 T1 T2");
        Serial.println("A,B,C,D,E (X-OFFSETS)");
        Serial.println("a,b,c,d,e (y-offsets)");
        Serial.println("O,o(verrun calib.)");
        Serial.println("L,l(earn coarse)");
        Serial.println("P,p(ump [0-255],fan speeds) P0 F0 F1)");
        Serial.println("X(periment toggle) fix direction, show mA");
        Serial.println("# print tables");
        break;

      case '#': // Print position tables
        Serial.println(Str+"+6mm\t+2mm\tCentre\t-2mm\t-6mm\t");
        Serial.println(Str+"a="+vert.table[0][0]+"\tb="+vert.table[0][1]+"\tc="+vert.table[0][2]+"\td="+vert.table[0][3]+"\te="+vert.table[0][4]+"\tReverse");
        Serial.println(Str+"a="+vert.table[1][0]+"\tb="+vert.table[1][1]+"\tc="+vert.table[1][2]+"\td="+vert.table[1][3]+"\te="+vert.table[1][4]+"\tForward");
        Serial.println(Str+"a="+vert.table[2][0]+"\tb="+vert.table[2][1]+"\tc="+vert.table[2][2]+"\td="+vert.table[2][3]+"\te="+vert.table[2][4]+"\tTarget" );
        Serial.println(Str+"A="+horz.table[0][0]+"\tB="+horz.table[0][1]+"\tC="+horz.table[0][2]+"\tD="+horz.table[0][3]+"\tE="+horz.table[0][4]+"\tReverse");
        Serial.println(Str+"A="+horz.table[1][0]+"\tB="+horz.table[1][1]+"\tC="+horz.table[1][2]+"\tD="+horz.table[1][3]+"\tE="+horz.table[1][4]+"\tForward");
        Serial.println(Str+"A="+horz.table[2][0]+"\tB="+horz.table[2][1]+"\tC="+horz.table[2][2]+"\tD="+horz.table[2][3]+"\tE="+horz.table[2][4]+"\tTarget" );
        break;

      case 'L': { // Learn coarse. Compound {} braces avoid compiler bug
        char  plc; // Place as a letter
        char *rot; // Rotation direction as a string
        int learns = LEARNS;
        int err[2][PLACES]; // Position errors
        int opt[2][PLACES]; // Optimum positions
        int pre[2][PLACES] = {{1023, 1023, 1023, 1023, 1023},  // Load previous error
                              {1023, 1023, 1023, 1023, 1023}}; // with worst case
        unsigned long tim;

        Serial.println(Str+"#Learn coarse over "+update(&learns,1,8)+" learns");
        Serial.println(Str+"#Loop\tRot.\tPlace\tCoarse\tTarget\tActual\tError\tTime[ms]");

        // Load initial guesses ...
        for ( int p=0; p < PLACES; p++ ) {
          a->table[REVERSE][p] = a->table[TARGET][p] + 5;
          a->table[FORWARD][p] = a->table[TARGET][p] - 5;
        }

        // ... and see if we can improve
        for ( int l=0; l < learns; l++ ) {
          for ( int r = REVERSE; r <= FORWARD; r++ ) {
            digitalWrite( a->ROTAT, r );
            for ( int p=0; p < PLACES; p++ ) {
              a->coarse = a->table[r][p];
              a->maxrun = millis() + MAXRUN;
              a->millis = millis();
              while ( a->coarse )
                move( a ); // To target
              tim = millis() - a->millis;
              delay( SETTLE );

              err[r][p] = a->table[TARGET][p] - (a->angle = analogRead(a->ANGLE));
              if ( abs(err[r][p]) > 20 ) { // Error too large
                Serial.print(Str+'#'+l+" BAD");
              } else {
                Serial.print(Str+' '+l);
                // Save if better then previous
                if ( abs(err[r][p]) < abs(pre[r][p]))
                  opt[r][p] = a->table[r][p];
                pre[r][p] = err[r][p];

                // Adjust coarse position by half the error
                a->table[r][p] += (err[r][p] * 0.5);
              }
              rot = r ? "FWD" : "REV";
              plc = isupper( a->cmd ) ? p+'A' : p+'a';
              Serial.println(Str+'\t'+rot+'\t'+plc+'\t'+a->table[r][p]+'\t'+a->table[TARGET][p]+'\t'+a->angle+'\t'+err[r][p]+'\t'+tim);
            }
          }
        }
        // Update coarse values with optimums
        for ( int r = REVERSE; r <= FORWARD; r++ )
          for ( int p=0; p < PLACES; p++ )
            a->table[r][p] = opt[r][p];

      } break;

      case 'O': {  // Nudge over-run
        // Deceleration varying nudge length
        int i   = 0;            // Index
        int mov = 0;            // Move during nudge
        int ovr = 0;            // Overrun after nudge
        int           an;       // Angle now
        int           as[1024]; // Angle saved
        unsigned long ts[1024]; // [ms] Time saved
        unsigned long tn;       // [ms] Time now 
        unsigned long tmo;      // [ms] Settling timeout
        unsigned long stl = 0;  // [ms] Settle time

        update(&a->nudge,1,255);      // Update nudge if supplied
        mov = analogRead(a->ANGLE);   // Angle now
        digitalWrite(a->BRAKE, LOW);  // Brake OFF
        delay(a->nudge);              // Nudge
        digitalWrite(a->BRAKE, HIGH); // Brake ON

        as[i] = analogRead(a->ANGLE); // First position
        ts[i] = tmo = millis();       // First timestamp
        tmo += 300;                   // Timeout
        while( (tn = millis()) < tmo ) {
          if ( as[i] != (an = analogRead(a->ANGLE))) {
            ts[++i] = tn;
            as[  i] = an;
          }
        }
        // Dump data - COMMENTED OUT
//        Serial.println(Str+"#Count\tTS[ms]\tSettle\tNudge="+a->nudge+" Rotat="+(a->rotat?"REV":"FWD"));
        for (int d=0 ; d < i; d++) {
          an = abs(as[d]-as[0]);
          tn = ts[d]-ts[0];
//          Serial.println(Str+d+'\t'+tn+'\t'+an );
          if ( an > ovr ) {
            ovr = an; // Max angle was reached ...
            stl = tn; // ... at this time
          }
        }
        // NOW ONLY PRINT RESULT
        mov -= as[0];    // Nudge move
        mov  = abs(mov); // Magniude of move
        Serial.println(Str+"Nudge="+a->nudge+" Move="+mov+" Overrun="+ovr+" for "+stl+" ms"+" Tot="+(mov+ovr));

      } break;

      case 'X': // Toggle experimental test mode
        test = !test;
        Serial.println( Str+"X "+test ); 
        break;

      case ' ': // Newline
        Serial.println(); 
        break;

      default: // Newline to show we are listening
        break;
    }
  }

  // Must be fast enough to read position within tolerance
  move ( &vert );
  move ( &horz );

  if ( !vert.coarse && !horz.coarse ) {
    nudge( &vert );
    nudge( &horz );
  }
}

// Coarse movement
// a = this axis
void move( axis_t *a ) {
  if (a->coarse) {
    if (reached(a->coarse, a->angle=analogRead(a->ANGLE), MOVE_TOL, NULL, NULL)) {
      digitalWrite(a->BRAKE, HIGH);  // Brake ON
      a->settle = millis()+DECEL;    // Delay before attempting nudge
      a->coarse = a->nudges = 0;     // Coarse phase done, onto nudging
    } else if (millis() > a->maxrun){// Run for too long
      digitalWrite(a->BRAKE, HIGH);  // Brake ON
      a->target = a->coarse = 0;     // Cancel move
      a->timed  = TIMEOUT;           // Mark as fault
    } else {
      digitalWrite(a->BRAKE, LOW);   // Brake OFF 
      if (test) // Display direction, angle and motor current
        Serial.println(Str+(millis()-a->millis)+' '+a->axis+' '+a->rotat+' '+a->angle+' '+((unsigned long)analogRead(a->POWER)*100000UL)/33769UL);
    }
  } else {
    digitalWrite(a->BRAKE, HIGH);    // Brake ON
  }
}

// Nudge movement
// a = this axis
void nudge( axis_t *a ) {
  // Nudging blocks so must be done after both coarse placements are complete
  // Nudge if target enabled and not waiting to settle
  if ( a->target && (millis() > a->settle) ) {
    // Lower tolerance if excess nudging going on
    a->tol = a->nudges >= EXCESS ? MOVE_TOL : FINE_TOL;

    if ( reached( a->target, a->angle=analogRead(a->ANGLE), a->tol, &a->rotat, &a->error )) {
      a->target = a->coarse = 0;         // Finished ..
      a->timed  = millis() - a->millis;  // .. in total time
    } else if ( a->nudges++ < NUDGES ) { // Some nudges left
      a->nudge = abs( a->error*SCALE );  // Proportional nudge ..
      digitalWrite( a->ROTAT, a->rotat );// .. in appropriate direction
      digitalWrite( a->BRAKE, LOW );     // Brake OFF
      delay( a->nudge );                 // Nudge. Blocks as must be precise
      digitalWrite( a->BRAKE, HIGH );    // Brake ON
      a->settle = millis() + SETTLE;     // Settling time before next nudge
    } else {
      digitalWrite( a->BRAKE, HIGH );    // Brake ON
      a->target = a->coarse = 0;         // Finished ..
      a->timed  = TIMEOUT;               // .. but not in time
    }
  }
}

// Is angle within tolerance of target?
// target = target angle
// angle  = current angle
// tol    = tolerance
// rot    = returned rotation direction
// err    = returned rotation error
// returns true, within tolerance, false, out of tolerance
bool reached(int target, int angle, int tol, int *rot, int *err) {
  if (rot && !test ) // Set direction if requested and not in experimental mode
    if ( target > angle )
      *rot = abs(angle-target) > abs(angle-target-1023) ? REVERSE : FORWARD;
    else
      *rot = abs(angle-target) < abs(angle-target+1023) ? REVERSE : FORWARD; 
 
// DEBUG: Force direction clockwise
//  *rot = REVERSE; 

  if (err) // If error requested
    *err = target - angle;  // Return position error

  return (angle <= (target + tol)) && (angle >= (target - tol));
}

// Read any supplied integer string into a value with clipping
// *val    = pointer to integer string, if not a number then no update
// min/max = limits, value is clipped
// returns updated or original value
int update(int *val, int min, int max ) {
  String buf = Serial.readStringUntil('\n');
  if (isDigit(buf.charAt(0))) 
    *val = constrain(buf.substring(0).toInt(), min, max);
  return *val;
}

// Read any supplied character string into a value with clipping
// *val    = pointer to int array , if not a char then no update
// min/max = limits, value is clipped
// returns updated or original value
int alpha_update(int *val, int min, int max) {
  String buf = Serial.readStringUntil('\n');
  if (isAlpha(buf.charAt(0)))
    *val = constrain(buf[0], min, max);
  return *val;
}

// Interrupt routine for pump RPM
void pump_isr(void) {
  pump_count++;
}

// Interrupt routine for pump RPM
void fan_isr(void) {
  fan_count++;
}
