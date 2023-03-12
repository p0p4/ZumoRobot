#include "includes.h"

//MAZE

void ToNextInt(void);
void Turn90(int direction);
void RecPosition(void);
void FollowLine(void);
bool OnInt(void);
bool Obstacle(int distance);
void Setup(void);
void ToStart(void);
void Shutdown(void);

#define LEFT 0
#define UP 0.5
#define RIGHT 1
#define WHITE 0
#define BLACK 1

struct sensors_ dig;

int speed = 255;
int tooClose = 10;              //object detecion distance
int xCord = 0, yCord = -1;      //starting coordinates
int varSide = LEFT;             //variable avoidance direction, left or right
float currentDir = UP;          //robot facing direction when starting
bool end = false;

void zmain(void)
{
    Setup();

    while (SW1_Read())
    {
        vTaskDelay(1);
    }

    ToStart();

    print_mqtt("Zumo030/ready", "maze");

    IR_wait();

    TickType_t startTime = xTaskGetTickCount();

    print_mqtt("Zumo030/start", "%d", startTime);

    while (!end)
    {
        if (currentDir == UP)               //if current direction is upward
        {
            if (yCord >= 11)
            {
                if (xCord > 0) Turn90(LEFT);
                if (xCord < 0) Turn90(RIGHT);

                while (xCord != 0)
                {
                    ToNextInt();
                }

                if (currentDir == LEFT) Turn90(RIGHT);
                if (currentDir == RIGHT) Turn90(LEFT);
                    
                while (!end)
                {
                    ToNextInt();
                }
            }
            else if (!Obstacle(tooClose))
            {
                ToNextInt();

                if (xCord > 0) varSide = LEFT;
                if (xCord < 0) varSide = RIGHT;
            }
            else if (Obstacle(tooClose))
            {
                Turn90(varSide);
            }
        }
        else if (currentDir == LEFT)        //if current direction is left
        {
            if (!Obstacle(tooClose))
            {
                ToNextInt();
            }

            if (Obstacle(tooClose) || xCord == -3)
            {
                varSide = RIGHT;
            }
            Turn90(RIGHT);
        }
        else if (currentDir == RIGHT)       //if current direction is right
        {
            if (!Obstacle(tooClose))
            {
                ToNextInt();
            }
            
            if (Obstacle(tooClose) || xCord == 3)
            {
                varSide = LEFT;
            }
            Turn90(LEFT);
        }
    }

    TickType_t finishTime = xTaskGetTickCount();

    print_mqtt("Zumo030/stop", "%d", finishTime);

    print_mqtt("Zumo030/time", "%d", (finishTime - startTime));

    Shutdown();
}

//################################//TO NEXT INTERSECTION
TickType_t intDuration = 0;
TickType_t intTime = 1;
bool firstInt = true;

void ToNextInt(void)
{
    TickType_t intStart = xTaskGetTickCount();

    while (OnInt())     
    {   //forward while on line
        motor_forward(speed, 1);
    }

    while (!OnInt() && (intDuration < intTime))     
    {   //follow line till intersection is detected or time limit is reached
        FollowLine();

        if (firstInt == false)  
        {   //current time duration between intersections
            intDuration = (xTaskGetTickCount() - intStart);
        }
    }

    if (firstInt == true)   
    {   //calculating time taken between the very first two lines
        intTime = (xTaskGetTickCount() - intStart); //
        firstInt = false;
    }

    if (!(intDuration < intTime))   
    {   //stopping maze if the time limit between intersections is reached
        end = true;
    }

    if (!end)       
    {   //centering robot on intersection by delaying stop
        vTaskDelay(intTime / 2);    //delaying by half the time it took between the first two lines
    }

    RecPosition();
    motor_forward(0, 0);
    vTaskDelay(100);
}

//################################//INTERSECTION TURNING
void Turn90(int direction)
{
    while (!(dig.L1 == WHITE && dig.R1 == WHITE))       //turning till both middle sensors see white
    {
        reflectance_digital(&dig);
        SetMotors(!direction, direction, 64, 64, 1);
    }
    while (!(dig.L1 == BLACK && dig.R1 == BLACK))       //turning till both middle sensors see black
    {
        reflectance_digital(&dig);
        SetMotors(!direction, direction, 64, 64, 1);
    }

    //updating current direction after turn
    if (direction == LEFT) currentDir -= 0.5;
    if (direction == RIGHT) currentDir += 0.5;
        
    motor_forward(0, 0);
}

//################################//RECORD POSITION
void RecPosition(void)
{   //keeping coordinates updated with the help of current direction
    if (currentDir == UP) yCord += 1;
    if (currentDir == LEFT) xCord -= 1;
    if (currentDir == RIGHT) xCord += 1;

    if (yCord > -1 && yCord < 14)
    {
        print_mqtt("Zumo030/position", "%d %d", xCord, yCord);
    }
}

//################################//LINE FOLLOWER
void FollowLine(void)
{
    reflectance_digital(&dig);

    if (dig.L1 == BLACK && dig.R1 == BLACK)
    {   //going straight if robot is centered on the track
        motor_forward(speed, 1);
    }
    else if (dig.R3 == BLACK || dig.R2 == BLACK || dig.R1 == BLACK)
    {   //turning right if robot is off center to the left
        motor_turn(speed, (speed * 0.75), 1);
    }
    else if (dig.L1 == BLACK || dig.L2 == BLACK || dig.L3 == BLACK)
    {   //turning left
        motor_turn((speed * 0.75), speed, 1);
    }
    else
    {
        motor_forward(0, 0);
    }
}

//################################//ON INTERSECTION SCAN
bool OnInt(void)
{
    bool state = false;

    reflectance_digital(&dig);

    if (dig.L1 == BLACK && dig.R1 == BLACK && (dig.L3 == BLACK || dig.R3 == BLACK))
    {
        state = true;
    }
    else
    {
        state = false;
    }

    return state;
}

//################################//OBSTACLE SCAN
bool Obstacle(int distance)
{
    bool state = false;

    if (Ultra_GetDistance() < distance)
    {
        state = true;
    }
    else
    {
        state = false;
    }

    return state;
}

//################################//SETUP
void Setup(void)
{
    //motors
    motor_start();
    motor_forward(0, 0);

    //reflectance sensors
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);

    //infrared receiver
    IR_Start();

    //Ultrasonic
    Ultra_Start();
}

//################################//TO STARTLINE
void ToStart(void)
{
    while (!OnInt())
    {
        motor_forward(128, 1);
    }
    motor_forward(0, 0);
}

//################################//SHUTDOWN
void Shutdown(void)
{
    //motors
    motor_forward(0, 0);
    motor_stop();

    while (true)
    {
        vTaskDelay(1);
    }
}