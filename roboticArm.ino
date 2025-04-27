#include <Arduino.h>

// Set up pins
const int motorStepPin_1 = 2;
const int motorDirPin_1 = 3;
const int motorEnablePin_1 = 4;
const int motorStepPin_2 = 5;
const int motorDirPin_2 = 6;
const int motorEnablePin_2 = 7;
const int MSPin_1 = 9;
const int MSPin_2 = 10;
const int MSPin_3 = 11;
const int blackButtonPin = A9;
const int greenButtonPin = A10;

// Variables
int currentStep_1 = 0;
int currentStep_2 = 0;
int desiredStep_1 = 0;
int desiredStep_2 = 0;
int currentPos = 0;
const int stepDelay = 5;   // Step pulse width
const int loopDelay = 50;  // Delay between steps

bool runFullTrajectory = false; // 标志位：是否执行完整轨迹
int current = 0; //组计数器
// Trajectory array {0, 0}开头，此为启动项 (这里只示例了几行，自己补全）
const int trajectory[][2] = 
{
  {0, 0}, {10, -17}, {2, -18}, {3, -18}, {4, -17}, {3, -16}, {6, -10}, {6, -10}, {6, -9},
  {5, -5}, {4, -3}, {3, -2}, {2, -1}, {1, 0}, {0, 1}, {-1, 2}, {-3, 3}, {-4, 4},
  {-6, 4}, {-5, 3}, {-7, 3}, {-9, 2}, {-11, 2}, {-12, 2}, {-14, 2}, {-13, 2}, {-15, 2},
  {-17, 2}, {-18, 2}, {-20, 2}, {-22, 2}, {-21, 2}, {-23, 3}, {-22, 3}, {57, 2}, {55, 2},
  {53, 2}, {52, 3}, {51, 3}, {51, 5}, {51, 6}, {51, 8}, {52, 10}, {59, 22}, {59, 24},
  {59, 26}, {59, 28}, {59, 29}, {59, 31}, {59, 33}, {59, 34}, {60, 36}, {60, 38}, {61, 40},
  {62, 41}, {62, 43}, {63, 45}, {64, 46}, {65, 47}, {89, 67}, {88, 68}, {87, 70}, {87, 71},
  {88, 72}, {88, 73}, {77, 84}, {75, 83}, {71, 80}, {69, 78}, {66, 75}, {61, 66}, {59, 63},
  {59, 62}, {61, 58}, {60, 58}, {61, 57}, {62, 52}, {63, 50}, {64, 48}, {65, 46}, {66, 45},
  {77, 35}, {76, 34}, {76, 33}, {76, 32}, {77, 32}, {66, 31}, {144, -1}, {142, 1}, {140, 3},
  {137, 5}, {135, 7}, {133, 9}, {131, 10}, {129, 12}, {126, 14}, {129, 12}, {128, 10}, {128, 6},
  {128, 4}, {61, -5}, {60, -4}, {58, -4}, {57, -4}, {56, -4}, {54, -4}, {53, -3}, {51, -3},
  {50, -3}, {48, -3}, {47, -3}, {45, -3}, {44, -3}, {42, -4}, {41, -4}, {39, -4}, {38, -4},
  {36, -5}, {34, -5}, {29, 4}, {30, 6}, {30, 8}, {31, 9}, {32, 11}, {32, 13}, {33, 14},
  {33, 16}, {34, 17}, {34, 19}, {34, 21}, {40, 48}, {41, 49}, {42, 51}, {43, 52}, {44, 53},
  {45, 54}, {46, 56}, {47, 57}, {48, 58}, {49, 59}, {50, 60}, {51, 61}, {52, 62}, {53, 63},
  {55, 65}, {57, 66}, {58, 67}, {59, 68}, {60, 73}, {54, 73}, {53, 74}, {52, 75}, {51, 75},
  {50, 76}, {49, 77}, {48, 77}, {47, 78}, {45, 79}, {44, 79}, {43, 80}, {42, 80}, {41, 81},
  {40, 81}, {39, 82}, {38, 82}, {37, 83}, {35, 83}, {34, 83}, {33, 84}, {32, 84}, {30, 84},
  {29, 84}, {27, 84}, {26, 84}, {24, 84}, {22, 84}, {21, 84}, {19, 84}, {18, 84}, {16, 84},
  {15, 84}, {13, 84}, {12, 84}, {10, 84}, {9, 84}, {7, 84}, {6, 84}, {4, 84}, {2, 84},
  {0, 84}, {-1, 84}, {-3, 84}, {-17, 72}, {-19, 69}, {-22, 61}, {-22, 60}, {-21, 59}, {-21, 55},
  {-22, 53}, {-28, 43}, {-30, 41}, {-33, 39}, {-35, 37}, {-36, 32}, {-34, 32}, {-33, 32},
  {-31, 32}, {-30, 32}, {-28, 32}, {-27, 31}
};

const int lastPos = sizeof(trajectory) / sizeof(trajectory[0]);

//结构体，保存电机所需参数
struct StepInfo 
{
  int stepNeed1;
  int dir1;
  int stepNeed2;
  int dir2;
};

StepInfo calStepInfo(int currentStep1, int nextStep1, int currentStep2, int nextStep2);

void runMotor(int motorStepPin, int motorDirPin);

void setup() 
{
  // Setup motor pins
  pinMode(motorDirPin_1, OUTPUT);
  pinMode(motorEnablePin_1, OUTPUT);
  pinMode(motorStepPin_1, OUTPUT);
  pinMode(motorDirPin_2, OUTPUT);
  pinMode(motorEnablePin_2, OUTPUT);
  pinMode(motorStepPin_2, OUTPUT);

  // Setup microstepping
  pinMode(MSPin_1, OUTPUT);
  pinMode(MSPin_2, OUTPUT);
  pinMode(MSPin_3, OUTPUT);
  digitalWrite(MSPin_1, LOW);
  digitalWrite(MSPin_2, HIGH); // 1/4步模式
  digitalWrite(MSPin_3, LOW);

  // Setup button pins
  pinMode(blackButtonPin, INPUT);
  pinMode(greenButtonPin, INPUT);

  // Enable motors
  digitalWrite(motorEnablePin_1, LOW);
  digitalWrite(motorEnablePin_2, LOW);

  Serial.begin(9600);
}

void RUNNING();

void loop()
{
  //按钮检测
  if (analogRead(greenButtonPin) < 2)
  {
    runFullTrajectory = true;
  }
  else if (analogRead(blackButtonPin) < 2)
  {
    RUNNING();
    delay(100);
  }

  //绿色按钮执行全部操作
  while (runFullTrajectory)
  {
    //执行1次操作
    RUNNING();
    
    //终止
    if (current >= lastPos - 1)
    {
      runFullTrajectory = false;
      break;
    }

  }
  delay(loopDelay);
}

//计算方向，步数
StepInfo calStepInfo(int currentStep1, int nextStep1, int currentStep2, int nextStep2)
{
  StepInfo info;
  info.stepNeed1 = nextStep1 - currentStep1;
  info.stepNeed2 = nextStep2 - currentStep2;
  info.dir1 = (info.stepNeed1 >= 0) ? 1 : -1;
  info.dir2 = (info.stepNeed2 >= 0) ? 1 : -1;
  return info;
}

//执行1步
void runMotor(int motorStepPin, int motorDirPin)
{
  digitalWrite(motorStepPin, HIGH);
  delay(stepDelay);
  digitalWrite(motorStepPin, LOW);
  delay(stepDelay);
}

void RUNNING()
{
  
  if (current >= lastPos - 1) 
  {
    return; // 防止越界
  }
  
  //计算所需步数
    int currentStep1 = trajectory[current][0];
    int nextStep1 = trajectory[current + 1][0];
    int currentStep2 = trajectory[current][1];
    int nextStep2 = trajectory[current + 1][1];


    StepInfo result = calStepInfo(currentStep1, nextStep1, currentStep2, nextStep2);

    // 拿到每个结果
    int motor1Steps = result.stepNeed1;
    int motor1Direction = result.dir1;
    int motor2Steps = result.stepNeed2;
    int motor2Direction = result.dir2;
    Serial.print("第");
    Serial.print(current);
    Serial.print("组变化量:");
    Serial.print(motor1Steps);
    Serial.print("   ");
    Serial.println(motor2Steps);
    //启动！
    //设置motor1，2方向
    int direction1 = (motor1Direction >= 0) ? HIGH : LOW;
    digitalWrite(motorDirPin_1, direction1);
    int direction2 = (motor2Direction >= 0) ? HIGH : LOW;
    digitalWrite(motorDirPin_2, direction2);
    
    //循环执行每一步
    // for (int i = 0; i < abs(motor1Steps); i++)
    // {
    //   runMotor(motorStepPin_1, motorDirPin_1);
    // }
    // for (int j = 0; j < abs(motor2Steps); j++)
    // {
    //   runMotor(motorStepPin_2, motorDirPin_2);
    // }
    
    //循环同步执行每一步
    int maxSteps = max(abs(motor1Steps), abs(motor2Steps));

    for (int i = 0; i < maxSteps; i++) {
      if (i < abs(motor1Steps)) {
        runMotor(motorStepPin_1, motorDirPin_1);
      }

      if (i < abs(motor2Steps)) {
        runMotor(motorStepPin_2, motorDirPin_2);
      }
    }

    //下一组
    current++;
}