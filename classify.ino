/* integration.ino
** Processing Integration sketch for SIXT33N Speech version
** 
** EE16B Spring 2016
** Emily Naviasky & Nathaniel Mailoa
**
*/

/*
** Created by William Smith, Pei Jei, and Dakota Sproch.
** Circuits by Girmai Legese and Anh Nguyen
*/

//define constants
#define MIC_INPUT P6_0
#define SIZE 2752
#define SIZE_AFTER_FILTER 172


/*---------------*/
/* CODE BLOCK A1 */
/*---------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE 80
#define PRELENGTH 5
#define THRESHOLD 0.7

#define KMEANS_THRESHOLD 0.06
#define LOUDNESS_THRESHOLD 700

/*----------------------*/
/* END OF CODE BLOCK A1 */
/*----------------------*/

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

//data array and index pointer
int i=0;
int re[SIZE]={0};
volatile int re_pointer = 0;

//timer period
float timer_ms = 0.35;

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
  
  
  //microphone ADC
  pinMode(MIC_INPUT, INPUT);
  pinMode(P6_1, INPUT);
  pinMode(P6_5, INPUT);
  //recording light
  pinMode(RED_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
  
  Serial.begin(38400); 
  Serial.println("I'm beginning!!!!");
}

void loop()
{
   
    
  if(re_pointer == SIZE){
    digitalWrite(RED_LED, LOW);
    Serial.println("I'm looping!!!!");

    // Apply enveloping function and get snippet with speech. Do classification only if loud enough
    if(envelope(re, result)){

      /*--------------*/
      /* CODE BLOCK B */
      /*--------------*/
      
      Serial.println("I'm running!!!!");

      // Perform principal component projection
      
      // subtract the mean
      for (int i = 0; i < SNIPPET_SIZE; i++)
      {
        result[i] = result[i] - mean_vec[i];
      }
      
      proj1 = 0;
      proj2 = 0;
      
      for (int i = 0; i < SNIPPET_SIZE; i++)
      {
        proj1 += result[i] * pca_vec1[i];
        proj2 += result[i] * pca_vec2[i]; 
      }

      // Classification - Hint: use the function l2_norm defined above
      // YOUR CODE HERE 
      
      /*---------------------*/
      /* END OF CODE BLOCK B */
      /*---------------------*/
      dist[0] = l2_norm(proj1, proj2, centroid1);
      dist[1] = l2_norm(proj1, proj2, centroid2);
      dist[2] = l2_norm(proj1, proj2, centroid3);
      dist[3] = l2_norm(proj1, proj2, centroid4);
      
      float minVal = dist[0];
      int minIdx = 0;
      
      for (int i = 1; i < 4; i++)
      {
        if (dist[i] < minVal) 
        {
          minIdx = i;
          minVal = dist[i];
        }
      }
      
      char * word1 = "Port";
      char * word2 = "Starboard";
      char * word4 = "Giddy up!";
      char * word3 = "Take it easy";
      char * words[4] = {word1, word2, word3, word4};
      char * word = words[minIdx];
      if (minVal <= KMEANS_THRESHOLD) 
      {
        Serial.print(word);
        Serial.print(", ");
        Serial.println(minVal * 1000);
      } else {
        Serial.println("Unknown");
      }

    }
    
    delay(2000);
    re_pointer = 0;
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



// Timer ISR for ADC sampling
#pragma vector=TIMER0_A0_VECTOR    // Timer A ISR
__interrupt void Timer1_A0_ISR( void )
{
  if(re_pointer < SIZE){
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for ADC
void setTimer(){
  // set the timer based on 25MHz clock
  TA0CCR0 = (unsigned int)(25000*timer_ms);       
  // enable interrupts for Timer A
  TA0CCTL0 = CCIE;             
  __bis_SR_register(GIE);
  TA0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
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

