#include <Arduino.h>

#include <util/atomic.h>

//tutorial from https://youtu.be/HRaZLCBFVDE?si=WYoHt1nGI-hfgg1aNq

//tutorial: https://youtu.be/eM4VHtettGg?si=zYYa_dooWAk8yBr1
template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 2*PI*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter structure: 
// LowPass<order> lp(cutoff frequency (Hz), sample frequency(Hz), adaptive (boolean flag))
LowPass<2> lp(fSet+0.1 ,1e3, true);

#define RPWM 5
#define LPWM 4
#define phaseA 2 //white wire
#define phaseB 3 //green wire

float prevT = 0; //previous time
float currT = 0; //current time
float deltaT = 0;
int prevCount = 0;  //previous count number
int currCount = 0;  //current count number
volatile int count_i = 0;

//filtered speed (filtered by a low-pass filter)
float vFbFilt_rps = 0;
float eInt = 0;
float edot = 0;

float pwrSpeed = 0;

//motor specs
float n = 1; //transmission ratio
float R = 1.83; //electric resistance [Ohm]
float K_m = 0.467; //motor torque constant [N.m/Amp]
float K_e = 56/60; //electromotive force constant [V/rad/sec]
float b_m = 2.58e-04; //motor viscous friction constant [N.m.s]
float I_m = 1.2e-04; //moment of inertia of the rotor [kg.m^2]

//load specs
float m = 1; //mass [kg]
float g = 9.81; //gravitational acceleration [m/s2]
float l = 0.1; //length [m]
float b = 0.1; //width [m]
float b_l = 0.1; //load viscous friction constant [N.m.s]

float I_l = 1/12 * m * (l*l + b*b); //moment of inertia of the object (solid block) [kg.m^2]
float w_l = m * g * l; //workload [kg.m^2/s^2]

float I_td = I_l + I_m * n * n;
float b_td = b_l + n * n * b_m * (K_m * K_e * n * n / R);
float k_u = K_m * n / R;

float alpha = I_td / k_u;

float u = 0;
float vSetDelay_rps = 0;

void readEncoder(){
  int inc = 0; //increment
  if(digitalRead(phaseB) == HIGH){
    inc = 1;
    }
  else{
    inc = -1;
    }
  count_i += inc;
  }

void setup() {
  //input pins to read encoder
  pinMode(phaseA,INPUT_PULLUP);
  pinMode(phaseB,INPUT_PULLUP);
  //output pins to control motor
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
    //Cu phap: attachInterrupt(interrupt, ISR, mode);
    //interrupt: số thứ tự của ngắt (int.0 và int.1 tương ứng chân D2 và D3)
    //Link tham khảo: http://arduino.vn/reference/attachinterrupt
  attachInterrupt(digitalPinToInterrupt(phaseA),readEncoder,RISING);
  Serial.begin(115200);
}



void controlMotor(float input, float pwrSpeed){
  if(input < 0){
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwrSpeed);
    }
  else if(input > 0){
    analogWrite(RPWM, pwrSpeed);
    analogWrite(LPWM, 0);
    }
  else{
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    }
  }

//function to read and filter feedback velocity
void readFbSpeed(){
  //read the position in an atomic block to avoid potential misreads
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    currCount = count_i;
    }

  float deltaT = currT - prevT;  //[s]

  int deltaCount = currCount - prevCount;
  // Serial.print(">deltaCount:");
  // Serial.println(deltaCount);

  float vFeedback_cps = deltaCount/ deltaT;  //[count/s]
  // Serial.print(">vFeedback_cps:");
  // Serial.println(vFeedback_cps);
  float vFeedback_rps = vFeedback_cps*2*PI/600.0; //[rad/s]

  //2nd-order ButterWorth filter (0.35Hz cutoff)
  vFbFilt_rps = lp.filt(vFeedback_rps);

  //update variables
  prevCount = currCount;
  prevT = currT;
  }

#define numCol 5

int col_vSet = 0;  //column's order of set velocity
int col_vFbFilt = 0;  //column's order of filtered feedback velocity
float valMax[2][numCol];  //array to store max value
float timeMax[2][numCol]; //array to store max time
float vFbFiltMax_rps = 0; //max value of feedback velocity
float vSetMax_rps = 0;  //max value of set velocity
float vSet_rps = 0;
float k = 0;
float tMax_vSet = 0;
float tMax_vFbFilt = 0;
int prevCycleOrder = 1;
int currCycleOrder = 1;
bool flag_readMax = false;
bool flag_countCycleOrder = false;

float kp = 200;
float ki = 0;
float kd = 1e-5;

int maxCycleOrder = 1;  //order of the cycle used to extract data
float fSet = 0.25;  //set frequency
float phase_delay = 0;
float amplitude = PI/4;  //max value of qSet (rad)

float currT_redefine = 0;

void findMax(int row, float order){

  //determine the order of set velocity's current cycle
  currCycleOrder = (floor(order)) + 1;  //read the integral part of n
    Serial.print(">cycle order:");
    Serial.println(currCycleOrder);
  //row = 0 means we're dealing with set velocity
  if (row == 0){ 
    if (currCycleOrder <= maxCycleOrder){
      flag_readMax = false;
      //determine if we're in the next cycle
      if (currCycleOrder > prevCycleOrder){
        //move to next column to save values for new cycle
        col_vSet++;
        //reset variables for the next cycle
        vSetMax_rps = 0;
        tMax_vSet = 0;                 
      else{
        //when we have found vSetMax (rad/s), store its value and occur time in defined arrays
        timeMax[row][col_vSet] = tMax_vSet;
        valMax[row][col_vSet] = vSetMax_rps;
          // Serial.print(">vSetMax_stored (rad/s):");
          // Serial.println(valMax[row][col_vSet]);
          Serial.print(">vSet_timeMax_stored (s):");
          Serial.println(timeMax[row][col_vSet]);
      }
    }
    else{
      flag_readMax = true; //set flag's value to TRUE after max value reading process is done
    }
  }
  }

  //row = 1 means we're dealing with filtered feedback velocity
  if (row == 1){ 
    if (currCycleOrder <= maxCycleOrder){
      //determine if we're in the next cycle
      if (currCycleOrder > prevCycleOrder){
        //move to next column to save values for new cycle
        col_vFbFilt++;
        //reset variables for the next cycle
        vFbFiltMax_rps = 0;
        tMax_vFbFilt = 0;
        //update cycle's order
        prevCycleOrder = currCycleOrder; 
      }
      if (vFbFilt_rps > vFbFiltMax_rps){ //find max value in 1 cycle
        vFbFiltMax_rps = vFbFilt_rps;
        tMax_vFbFilt = currT;
        // Serial.print(">vFbFiltMax (rad/s):");
        // Serial.println(vFbFiltMax_rps);
      }
      else{
        //when we have found vSetMax (rad/s), copy its value and occur time to defined arrays
        timeMax[row][col_vFbFilt] = tMax_vFbFilt;
        valMax[row][col_vFbFilt] = vFbFiltMax_rps;
          // Serial.print(">vFbFiltMax_stored (rad/s):");
          // Serial.println(valMax[row][col_vFbFilt]);
          Serial.print(">vFbFilt_timeMax (s):");
          Serial.println(timeMax[row][col_vFbFilt]);
      }
    }
  }
}

float phi = 0;
float phase = omega*currT + phi;
float omega = 2*PI*fSet;
float q0 = 0;

float qSet = 0;
float aSet = 0;
float qFb = 0;
float e = 0;
float currT0 = 0;
float pwrMaxSpeed = 100;

void loop() {
  currT = micros()/1e6; //current time [s]
    Serial.print("loop");
    Serial.println(currT);

  //redefine starting point of vSet
  if(flag_countCycleOrder == false && abs(cos(phase)-1) < 1e-3){
    flag_countCycleOrder = true;
    currT0 = currT;      
  }
  if(flag_countCycleOrder == true){
    currT_redefine = currT - currT0;
    if(flag_readMax == false){
      //with vSet = A*cos(k*pi + phi)
      //set n = (k/2 + 1/4 + phi/(2*pi))
      //every time n being an integer, we complete 1 full cycle of vSet 
      float n = (2*fSet*currT_redefine)/2 + 1/4  + phi/(2*PI);
        // Serial.print(">n:");
        // Serial.println(n);
      phase_delay = 0;

      //find max values and max time of set velocity
      findMax(0, n);

      //pwrSpeed = pow((vSet_rps/amplitude/omega),2) * pwrMaxSpeed;
        // Serial.print(">pwrSpeed:");
        // Serial.println(pwrSpeed);
      //control motor
      //controlMotor(vSet_rps, pwrSpeed);

      //find max values and max time of filtered feedback velocity
      findMax(1, n);
    }
    else{ //if flag_readMax == true
      //create a copy of set velocity with same phase as the filtered feedback velocity
      phase_delay = 2*PI*fSet*abs(timeMax[0][maxCycleOrder-1]-timeMax[1][maxCycleOrder-1]);
      }
    }
  //read and filter feedback speed
  readFbSpeed();
  
  qSet = amplitude*sin(phase) + q0;  //[rad]
  vSet_rps = amplitude*omega*cos(phase - phase_delay); //[rad/s]
  aSet = -amplitude*pow(omega, 2)*sin(phase); //[rad/s^2]
  
  qFb = currCount*2*PI/600.0; //600 is the number of counts per revolution
  e = qSet - qFb;
  eInt += e*deltaT;
  edot = vSet_rps - vFbFilt_rps;

  float u_control = aSet + kp*e + ki*eInt + kd*edot;
  float beta = (b_td * vFbFilt_rps + w_l * cos(qFb)) / k_u;
  u = alpha * u_control + beta;
  pwrSpeed = fabs(u/5.0 * pwrMaxSpeed);
  //ensure that pwrSpeed won't exceed pwrMaxSpeed
  if(pwrSpeed > pwrMaxSpeed){
    pwrSpeed = pwrMaxSpeed;
  }
  //control motor
  controlMotor(e, pwrSpeed);

    // Serial.print(">currCount:");
    // Serial.println(currCount);
    // Serial.print(">qSet (rad):");
    // Serial.println(qSet);
    Serial.print(">vSet (rad/s):");
    Serial.println(vSet_rps);
    // Serial.print(">aSet (rad/s^2):");
    // Serial.println(aSet);
    Serial.print(">phase delay:");
    Serial.println(phase_delay);
    // Serial.print(">qFeedback:");
    // Serial.println(qFb);
    Serial.print(">vSetDelay (rad/s):");
    Serial.println(vSetDelay_rps);
    // Serial.print(">pwrSpeed:");
    // Serial.println(pwrSpeed);
    Serial.print(">e:");
    Serial.println(e);
    Serial.print(">edot:");
    Serial.println(edot);
    Serial.print(">vFbFilt (rad/s):");
    Serial.println(vFbFilt_rps);

    delayMicroseconds(200);
}