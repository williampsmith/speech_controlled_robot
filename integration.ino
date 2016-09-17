/* integration.ino
** Final sketch for SIXT33N Speech version
** 
** EE16B Spring 2016
** Emily Naviasky & Nathaniel Mailoa
**
*/

// Define pins
// Note that if you change the encoder pins
// you also need to change the setup function,
// the end of the loop function and the ISR to 
// enable appropriate pin interrupts - ask your 
// GSI if you want to do this
#define LEFT_MOTOR P2_0
#define LEFT_ENCODER P2_5
#define RIGHT_MOTOR P1_5
#define RIGHT_ENCODER P1_2
#define MIC_ADC P6_0

// Define constants
#define TIMESTEP 200
#define HIGH_PWM 150
#define LOW_PWM 80
#define SIZE 2752
#define SIZE_AFTER_FILTER 172
#define INITIAL_PWM 130 // you might need to change this


/*---------------*/
/* CODE BLOCK A1 */
/*---------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE 80
#define PRELENGTH 5
#define THRESHOLD 0.7

#define KMEANS_THRESHOLD 0.04
#define LOUDNESS_THRESHOLD 1100

/*----------------------*/
/* END OF CODE BLOCK A1 */
/*----------------------*/


// Operation modes
#define MODE_LISTEN 0
#define MODE_DRIVE 1

boolean TIMER_MODE = MODE_LISTEN;
//int i=0;


// Timer period
#define TIMER_MS 0.35


/*---------------*/
/* CODE BLOCK A2 */
/*---------------*/

//define arrays
float pca_vec1[SNIPPET_SIZE] = { 0.053230540233,0.0539573569926,0.0601063291205,0.0790760206129,0.103961804957,0.118665663366,0.145684381118,0.128061563722,0.179245581683,0.193752190833,0.229262384467,0.222144484071,0.232064704367,0.23544840365,0.22463617806,0.214561928766,0.24460389629,0.208104371962,0.197342761799,0.146871516459,0.143562429387,0.087801796009,0.0787646770924,0.0494520752183,0.00950178818987,0.00963849716546,-0.014218755908,-0.0624124485436,-0.0804703196241,-0.118050159608,-0.11949971548,-0.139354665556,-0.134730848291,-0.115025788166,-0.116685710079,-0.105767336176,-0.102251014689,-0.0845552086427,-0.0952963814939,-0.114792327887,-0.105465722003,-0.129431440998,-0.121910765583,-0.124015138955,-0.116530671843,-0.120738949786,-0.109606073699,-0.123023372571,-0.0988218095262,-0.0938313825473,-0.0881953829034,-0.0894034098312,-0.0787983471865,-0.0738452196738,-0.062084708251,-0.053968126731,-0.0604592742566,-0.0478058039545,-0.0483639470172,-0.0379776293096,-0.0414657948991,-0.0391746261482,-0.0374856074393,-0.0387388902604,-0.0299697876876,-0.0327241495242,-0.0348774087719,-0.0277868959683,-0.0287880281203,-0.0190360320997,-0.0210948596324,-0.0249830824667,-0.0150421128415,-0.0106094137405,-0.0175485245494,-0.0149728472929,-0.0118327101204,-0.0111223059571,-0.000760538578151,-0.00410185272116
 }; 
                                
float pca_vec2[SNIPPET_SIZE] = { 0.0956131088028,0.0986844598844,0.105457066916,0.0807722802993,0.0558634555567,0.0310638804684,0.0760252920918,0.052429207373,0.147019651641,0.187807367944,0.177139614909,0.126961254569,0.134820517455,0.0853627361714,0.0302632665512,-0.0667179119318,-0.099414679264,-0.179992895505,-0.164500316682,-0.25089807577,-0.27069164117,-0.279051095565,-0.252430346503,-0.246281299446,-0.259099277491,-0.243270140585,-0.186920950063,-0.138760969646,-0.086735906528,-0.0547402135616,-0.0207161072585,-0.00811364170448,-0.0304692893009,-0.0492776776843,-0.0778116322119,-0.0881345928818,-0.0846149223571,-0.131427066597,-0.105847034479,-0.106933570776,-0.0570491864849,-0.0160550523715,-0.0114105834031,0.00382156028768,0.0355949646052,0.0266542137071,0.0398086941519,0.0218755780185,0.0427132278621,0.0608913085653,0.0503346360216,0.0946165838154,0.0525858355594,0.0634523616488,0.0603260285574,0.080813636444,0.0643133080441,0.0805175162113,0.0804929430102,0.0862869943855,0.0846791347702,0.0741606257888,0.0924310410181,0.0876117033689,0.0879295258851,0.0950157489421,0.0841052382417,0.0774834647966,0.083477151908,0.0837652777373,0.0674114695661,0.0616680308083,0.0470006213908,0.0339274202784,0.0282289843616,0.00570436119092,0.0159806183008,0.014749323944,0.000674517478193,0.0109792659165


 }; 
                                
float mean_vec[SNIPPET_SIZE] = { 0.00722609597416,0.00910714642686,0.0114435022428,0.0154367424884,0.0196437991347,0.0273313594191,0.0268747662055,0.0276687309573,0.0261876168096,0.0259585262557,0.0235197775495,0.0225164456542,0.0212352482413,0.0191873486912,0.0191697460651,0.0181213417886,0.0190610251166,0.0187623648057,0.01929089156,0.0183399407437,0.0196917808031,0.0179556154076,0.0170994564036,0.0160560708593,0.0159539607921,0.0149240407286,0.015224443568,0.0153704094574,0.0149622083413,0.016036272283,0.0151665602852,0.01585987154,0.0148175288391,0.0137908719149,0.0135175565124,0.0125814940557,0.0121665774684,0.0112924005815,0.0108076954751,0.0113754733631,0.0104469519845,0.0110329558666,0.0111809835981,0.0105331195226,0.0101853048717,0.0105807081622,0.0103667297756,0.010790991358,0.010169994124,0.00991784162654,0.0101711115028,0.0103401828511,0.00958879379773,0.00910045107252,0.00908316681909,0.00825556235157,0.00823269939601,0.0083277899947,0.00783803096221,0.00797183445944,0.00752657879135,0.00726861219353,0.00693723637348,0.00685792022549,0.00606731183544,0.00668188984342,0.00634196221688,0.00594625155463,0.00611725539715,0.00613026032884,0.00553141008229,0.0056348446377,0.00532329846042,0.00499273045235,0.00492285109009,0.00496788437657,0.0045302900063,0.00469735726625,0.00447456300339,0.00420158295821 };
               
               
float centroid1[2] = { 0.06957015 , 0.005142 }; 
float centroid2[2] = { 0.00809837 , -0.00998368 }; 
float centroid3[2] = { -0.04171452 , 0.03285631 };
float centroid4[2] = { -0.03691408 , -0.01786947 };

/*----------------------*/
/* END OF CODE BLOCK A2 */
/*----------------------*/


float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float dist[4] = {0};


// Data array and index pointer
int i=0;
int re[SIZE]={0};
volatile int16_t re_pointer = 0;

//Control variables
boolean do_loop = 0;
float left_target_speed = 0.04;
float left_current_speed = 0; // in ms
long left_position = 0;
long left_last_time = 0;
int32_t left_history = 0;
int left_num_ticks = 0;

float right_target_speed = 0.04;
float right_current_speed = 0; // in ms
long right_position = 0;
long right_last_time = 0;
int32_t right_history = 0;
int right_num_ticks = 0;

long tempr, templ = 0;
float left_cur_pwm = (HIGH_PWM + LOW_PWM)/2;
float right_cur_pwm = (HIGH_PWM + LOW_PWM)/2;


/*---------------*/
/* CODE BLOCK A3 */
/*---------------*/
boolean turningRight = false;
int rightLoops = 0;
boolean turningLeft = false;
int leftLoops = 0;



// Control vector
float F1_left = 1.73488;
float F2_left = 3940.6352;
float F1_right = 2.1071;
float F2_right = 4161.9871;

/*----------------------*/
/* END OF CODE BLOCK A3 */
/*----------------------*/


// drive_counter for how many times timestep since reset
int drive_count = 0;

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int*, float*);

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid){
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid){
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup()
{  
  // Left wheel control and encoder
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  
  // Right wheel control and encoder
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  //microphone ADC
  pinMode(MIC_ADC, INPUT);
  
  pinMode(RED_LED, OUTPUT);

  // Turn on and set pin interrupts
  // If you change your encoder pins this block
  // needs to be modified
  P2IE |= BIT5; // P2.5 interrupt enabled
  P2IES = BIT5; // set to high edge
  P2IFG &= ~BIT5; // P2.5 IFG cleared
  P1IE |= BIT2; // P1.2 interrupt enabled
  P1IES = BIT2; // set to high edge
  P1IFG &= ~BIT2; // P1.2 IFG cleared

  Serial.begin(38400);

  // Stop wheels
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0); 
  
  reset_blinker();
  
  // Set timer for timestep
  setTimer(MODE_LISTEN);
  __enable_interrupt();

  re_pointer = 0;
}



void loop()
{  
  if(TIMER_MODE == MODE_LISTEN && re_pointer == SIZE){  
    // Stop motor
    analogWrite(LEFT_MOTOR, 0);
    analogWrite(RIGHT_MOTOR, 0); 
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if(envelope(re, result)){

      /*--------------*/
      /* CODE BLOCK B */
      /*--------------*/

      Serial.println("I'm running!!!!");

      // Perform principal component projection
      
      // subtract the mean
      for (int k = 0; k < SNIPPET_SIZE; k++)
      {
        result[k] = result[k] - mean_vec[k];
      }
      
      proj1 = 0;
      proj2 = 0;
      
      for (int k = 0; k < SNIPPET_SIZE; k++)
      {
        proj1 += result[k] * pca_vec1[k];
        proj2 += result[k] * pca_vec2[k]; 
      }

      // Classification - Hint: use the function l2_norm defined above
      // YOUR CODE HERE 
      
      dist[0] = l2_norm(proj1, proj2, centroid1);
      dist[1] = l2_norm(proj1, proj2, centroid2);
      dist[2] = l2_norm(proj1, proj2, centroid3);
      dist[3] = l2_norm(proj1, proj2, centroid4);
      
      float minVal = dist[0];
      int minIdx = 0;
      
      for (int k = 1; k < 4; k++)
      {
        if (dist[k] < minVal) 
        {
          minIdx = k;
          minVal = dist[k];
        }
      }
      
      char * word1 = "Port";
      char * word2 = "Starboard";
      char * word3 = "Take it easy";
      char * word4 = "Giddy up!";
      char * words[4] = {word1, word2, word3, word4};
      char * word = words[minIdx];
      if (minVal <= KMEANS_THRESHOLD) 
      {
        Serial.print(word);
        Serial.print(", ");
        Serial.println(minVal * 1000);
      } else {
        Serial.println("Unknown");
        minIdx = -1;
      }
      
      if(minIdx == 0) {
          wheel_drive(.06,.06);
          turnLeft();
      }
      else if(minIdx == 1) {
          wheel_drive(.06,.06);
          turnRight();
      }
      else if(minIdx == 2)
     {
        wheel_drive(.05,.05);
     } 
     else if(minIdx == 3)
     {
        wheel_drive(.10,.10);
     }    
     
      /*---------------------*/
      /* END OF CODE BLOCK B */
      /*---------------------*/

    }

    delay(2000);
    re_pointer = 0;
  }
 
  else if(TIMER_MODE == MODE_DRIVE && do_loop){
    
    // Update speed data
    if (left_num_ticks > 0){
      left_current_speed = (float)left_num_ticks/(float)left_history;
    } else {
      left_current_speed = 1.0/200;
    }
    left_history = 0;
    left_num_ticks = 0;

    if (right_num_ticks > 0){
      right_current_speed = (float)right_num_ticks/(float)right_history;
    } else {
      right_current_speed = 1.0/200;
    }
    right_history = 0;
    right_num_ticks = 0;
    
    //Serial.print(left_current_speed);
    //Serial.print('\t');
    //Serial.print(left_position);
    //Serial.print('\t');
    //Serial.print(right_current_speed);
    //Serial.print('\t');
    //Serial.println(right_position);
    

    /*--------------*/
    /* CODE BLOCK C */
    /*--------------*/

    // Input into open loop (u: change PWM)
    // YOUR CODE HERE
    processRightTurn();
    processLeftTurn();
    
    input_left(F1_left * (right_position - left_position) + F2_left * (left_target_speed - left_current_speed));
    input_right(F1_right * (left_position - right_position) + F2_right * (right_target_speed - right_current_speed));

    /*---------------------*/
    /* END OF CODE BLOCK C */
    /*---------------------*/


    // Send new PWM values
    analogWrite(LEFT_MOTOR, (int)left_cur_pwm);
    analogWrite(RIGHT_MOTOR, (int)right_cur_pwm); 
    
    // Counter for how many times loop is executed since entering DRIVE MODE
    drive_count++;

    if (drive_count == 4*1000/TIMESTEP){
      // Completely stop and go back to LISTEN MODE after 4 seconds
      re_pointer = 0;
      analogWrite(LEFT_MOTOR, 0);
      analogWrite(RIGHT_MOTOR, 0); 
      delay(1000); // 1 second buffer for wheels to stop
      TIMER_MODE = MODE_LISTEN;
      setTimer(MODE_LISTEN);
    }
             
    do_loop = 0;  
  }
  
  // Encoder reading for wheel not moving
  // Needs to be modified if encoder pins changed
  long temp1 = millis();
  if (temp1 - right_last_time > 1000) {
    P1IFG |= BIT2;
  }
  if (temp1 - left_last_time > 1000) {
    P2IFG |= BIT5;
  }

}


// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out){
  int32_t avg = 0;
  float maximum = 0;
  int thres_index = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++){
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }
  
  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) return false;

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) 
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  for (int i = 0; i < SNIPPET_SIZE; i++){
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out 
  for (int i = 0; i < SNIPPET_SIZE; i++)
    data_out[i] = data_out[i] / total;

  return true;
}


// Helper function to set new target velocities and change to DRIVE MODE
void wheel_drive(float left_vel, float right_vel){
  // Reset variables
  drive_count = 0;
  left_position = 0;
  right_position = 0;
  TIMER_MODE = MODE_DRIVE;
  left_target_speed = left_vel;
  right_target_speed = right_vel;

  // Enter DRIVE MODE
  setTimer(MODE_DRIVE);

  // Send starting pulse to start motor movement
  left_cur_pwm = INITIAL_PWM;
  right_cur_pwm = INITIAL_PWM;
  if (left_vel > 0) analogWrite(LEFT_MOTOR, left_cur_pwm);
  if (right_vel > 0) analogWrite(RIGHT_MOTOR, right_cur_pwm); 

  // Trigger encoder readings
  right_history = 0;
  right_num_ticks = 0;
  left_history = 0;
  left_num_ticks = 0;
  P1IFG |= BIT2;
  P2IFG |= BIT5;
  delay(200);
}


// New PWM signal for left controller
void input_left(float in){
  left_cur_pwm += in;
  // Protect for saturation
  if(left_cur_pwm > 255) left_cur_pwm = 255;
  if(left_cur_pwm < 0) left_cur_pwm = 0;
}

// New PWM signal for right controller
void input_right(float in){
  right_cur_pwm += in;
  // Protect for saturation
  if(right_cur_pwm > 255) right_cur_pwm = 255;
  if(right_cur_pwm < 0) right_cur_pwm = 0;
}

// Port 2 ISR for left encoder
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
  if (P2IFG & BIT5){
    templ = millis();
    left_history += templ - left_last_time;
    left_last_time = templ;
    left_position += 1; //cm
    left_num_ticks += 1;
    P2IFG &= ~BIT5; // P2.5 IFG cleared
  }
}

// Port 1 ISR for right encoder
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  if (P1IFG & BIT2){
    tempr = millis();
    right_history += tempr - right_last_time;
    right_last_time = tempr;
    right_position += 1; //cm
    right_num_ticks += 1;
    P1IFG &= ~BIT2; 
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode){
  if (mode == MODE_LISTEN){
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int)(25000*TIMER_MS);  
    TA2CCTL0 = CCIE;             
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  } else if (mode == MODE_DRIVE){
    // Set the timer based on 32kHz clock
    TA2CCR0 = (unsigned int)(32.768*TIMESTEP);
    TA2CCTL0 = CCIE;             // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  TIMER_MODE = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR( void )
{
  if (TIMER_MODE == MODE_LISTEN){
    if(re_pointer < SIZE){
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_ADC) >> 4) - 128;
      re_pointer += 1;
    }
  } else if (TIMER_MODE == MODE_DRIVE){
    do_loop = 1;
  }
}


void reset_blinker(){
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}




void turnRight()
{
    Serial.println("Turn Right!!!");
   if(!turningRight) {
     turningRight = true;
     digitalWrite(GREEN_LED, HIGH); 
     rightLoops = 10;
      right_position += 40;
      
   }
}

void processRightTurn()
{
    if (turningRight){
        rightLoops--;
        if(rightLoops < 0) {
           turningRight = false;
           digitalWrite(GREEN_LED, LOW);        
         }
    }
}

void turnLeft()
{
  Serial.println("Turn Left!!!");
   if(!turningLeft) {
     turningLeft = true;
     digitalWrite(GREEN_LED, HIGH); 
     leftLoops = 10;
      left_position += 40;
      
   }
}

void processLeftTurn()
{
    if (turningLeft){
        leftLoops--;
        if(leftLoops < 0) {
           turningLeft = false;
           digitalWrite(GREEN_LED, LOW);        
         }
    }
}



