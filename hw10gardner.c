/**************************************************
 * CMPEN 473, Spring 2022, Penn State University
 *
 * Homework 10
 * On 04/20/2022
 * By Preston Gardner
 *
 ***************************************************/

/* Homework 9
 * User input from terminal controls robot:
 * In either mode:
 *      (1) Print IMU data: 'p'
 *      (2) Display distance and average speed: 'n'
 *      (3) Print map of car movement since last recording: 't'
 *      (4)
 *
 * mode 'm1':
 *      (1) Stop: ' s '
 *      (2) Forward: ' w '
 *      (3) Backward: ' x '
 *      (4) Faster: ' i ' 5% PWM power increase for each ‘i’ key hit
 *      (5) Slower: ' j ' 5% PWM power decrease for each ‘j’ key hit
 *      (6) Left: ' a ' 15 degree turn for each ‘a’ key hit, smooth transition
 *      (7) Right: ' d ' 15 degree turn for each ‘d’ key hit, smooth transition
 *      (8) Quit: ' q ' to quit all program proper (without cnt’l c, and without an Enter key)
 *
 * mode 'm2':
 *      Car follows laser pointer autonomously. Press 'w' to start or 'm1' to switch back to manual driving mode
 *
 * Raspberry Pi 3 computer with
 * Motor driver controlled by
 * PWM on GPIO12 -> PWMA (left motors) and GPIO13 -> PWMB (right motors)
 * GPIO05 -> AI1, GPIO06 -> AI2 (left motor direction control)
 * GPIO22 -> BI1, GPIO23 -> BI2 (right motor direction control)
 *
 * IMPORTANT! This project is made for the raspberry pi 3. The base memory
 * address in io_peripherals.c needs to be 0x3F000000 to run on rasp pi 3
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "blue_stack.h"
#include "pwm_init.h"
#include "6050Init.h"
#include "float_queue.h"
#include "raspicam_wrapper.h"

#define PI 3.14159265358979
#define IMG_RED_HEIGHT 260
#define IMG_RED_WIDTH 320

const int PWM_MAX = 1000;
const int PWM_MIN = 260;
const int PWM_RANGE = PWM_MAX - PWM_MIN;
const int PWM_50_PERC = PWM_MIN + PWM_RANGE / 2;
const int PWM_5_PERC = PWM_RANGE / 20;
const int RIGHT_MAX = 926;
const int TURN_LOW = PWM_MIN + 2 * PWM_5_PERC;
const int RAMP_TIME = 15; // us interval to ramp up / down speed
const int INSTR_QUEUE_SIZE = 1024;
const int DATA_QUEUE_SIZE = 10000; // about 1.67 minutes of data
const int HIST_QUEUE_SIZE = 50000; // about 8.35 minutes of data
const long TURN_TIME = 300000;
const long Z_C_TURN_TIME = TURN_TIME / 2;
const long CHECK_IR_INT = TURN_TIME / 100;
const long THREAD_PROCESS_TIME_uS = 10000;      // 10000 uS = 10 ms
const long THREAD_PIC_PROCESS_TIME_uS = 100000; // 100000 uS = 100 ms
const long CLOCKS_PER_MILLI = CLOCKS_PER_SEC / 1000;
const long CLOCKS_PER_MIRCO = CLOCKS_PER_MILLI / 1000;
const float ACC_PRECISION = 0.001;

uint64_t leftLookup[1000];
unsigned char *rawImg;

float xVelocity = 0, yVelocity = 0, heading = 0; // m/s
float xPos = 0, yPos = 0;                        // m

struct io_peripherals *io;
struct calibration_data calibration_accelerometer;
struct calibration_data calibration_gyroscope;
struct calibration_data calibration_magnetometer;
struct raspicam_wrapper_handle *Camera;

pthread_mutex_t alarmLock1, alarmLock2, inputLock, dataCollectLock, leftControlLock, rightControlLock, scheduleLock, pwmLock, printLock, takePicLock, fileWriteLock, initLock, newPicLock; // FIXME PRINTLOCK
pthread_cond_t willTakePic = PTHREAD_COND_INITIALIZER;
pthread_cond_t willCollectData = PTHREAD_COND_INITIALIZER;
pthread_cond_t dataCond = PTHREAD_COND_INITIALIZER;

struct intStack
{
    int32_t head;
    int32_t maxSize;
    int *contents;
};

struct publicState
{
    bool end;
    bool stopPics;
    bool stopCollection;
    char rightDirection;
    char leftDirection;
    int16_t left_lvl;
    int16_t right_lvl;
    struct queue leftQueue;
    struct queue rightQueue;
    struct queue inputQueue;
    struct spatial_data sd;
    sig_atomic_t readCounts;
    uint8_t mode;        // 0 = m0, 1 = m1, 2 = m2
    sig_atomic_t newPic; // is there a new pic to process?
    bool printPic;       // should we print pic?
    unsigned char image[IMG_RED_WIDTH][IMG_RED_HEIGHT];
    int16_t xDist;
    int16_t yDist;
    uint8_t alarmCounter;
    struct intStack rightHist;
    struct intStack leftHist;
} state;

struct dataQueue
{
    struct fQueue ACCEL_X;
    struct fQueue ACCEL_Y;
    struct fQueue ACCEL_Z;
    struct fQueue GYRO_X;
    struct fQueue GYRO_Y;
    struct fQueue GYRO_Z;
    struct fQueue TIME;
} dQueues;

struct RGB_pixel
{
    unsigned char R;
    unsigned char G;
    unsigned char B;
};

void initDataQueues(struct dataQueue *dq)
{
    initFQueue(&(dq->ACCEL_X), DATA_QUEUE_SIZE);
    initFQueue(&(dq->ACCEL_Y), DATA_QUEUE_SIZE);
    initFQueue(&(dq->ACCEL_Z), DATA_QUEUE_SIZE);
    initFQueue(&(dq->GYRO_X), DATA_QUEUE_SIZE);
    initFQueue(&(dq->GYRO_Y), DATA_QUEUE_SIZE);
    initFQueue(&(dq->GYRO_Z), DATA_QUEUE_SIZE);
    initFQueue(&(dq->TIME), DATA_QUEUE_SIZE);

    // float AX[DATA_QUEUE_SIZE];
    // float AY[DATA_QUEUE_SIZE];
    // float AZ[DATA_QUEUE_SIZE];
    // float GX[DATA_QUEUE_SIZE];
    // float GY[DATA_QUEUE_SIZE];
    // float GZ[DATA_QUEUE_SIZE];
    // float RT[DATA_QUEUE_SIZE];
    // float RH[DATA_QUEUE_SIZE];
    // float LH[DATA_QUEUE_SIZE];

    // memset(AX, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(AY, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(AZ, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(GX, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(GY, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(GZ, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(RT, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(RH, 0, DATA_QUEUE_SIZE * sizeof(float));
    // memset(LH, 0, DATA_QUEUE_SIZE * sizeof(float));

    dq->ACCEL_X.contents = calloc(0, DATA_QUEUE_SIZE * sizeof(float));
    dq->ACCEL_Y.contents = calloc(0, DATA_QUEUE_SIZE * sizeof(float));
    dq->ACCEL_Z.contents = calloc(0, DATA_QUEUE_SIZE * sizeof(float));
    dq->GYRO_X.contents = calloc(0, DATA_QUEUE_SIZE * sizeof(float));
    dq->GYRO_Y.contents = calloc(0, DATA_QUEUE_SIZE * sizeof(float));
    dq->GYRO_Z.contents = calloc(0, DATA_QUEUE_SIZE * sizeof(float));
    dq->TIME.contents = calloc(0, DATA_QUEUE_SIZE * sizeof(float));
}

void freeDataQueues(struct dataQueue *dq)
{
    free(dq->ACCEL_X.contents);
    free(dq->ACCEL_Y.contents);
    free(dq->ACCEL_Z.contents);
    free(dq->GYRO_X.contents);
    free(dq->GYRO_Y.contents);
    free(dq->GYRO_Z.contents);
    free(dq->TIME.contents);
}

void initStack(struct intStack *s, uint32_t maxSize)
{
    s->head = -1;
    s->maxSize = maxSize;
}

void clearStack(struct intStack *s)
{
    s->head = -1;
}

void stackPush(struct intStack *s, int val)
{
    if (s->head + 1 <= s->maxSize)
    {
        s->head++;
        s->contents[s->head] = val;
    }
    else
        printf("error: stack exceeds max size");
}

int stackPop(struct intStack *s)
{
    if (s->head - 1 >= -1)
    {
        int retVal = s->contents[s->head];
        s->head--;
        return retVal;
    }
    else
    {
        return -1;
        printf("error: negative indexing stack");
    }
}

void clearDataQueues(struct dataQueue *dq)
{
    clearFQueue(&(dq->ACCEL_X));
    clearFQueue(&(dq->ACCEL_Y));
    clearFQueue(&(dq->ACCEL_Z));
    clearFQueue(&(dq->GYRO_X));
    clearFQueue(&(dq->GYRO_Y));
    clearFQueue(&(dq->GYRO_Z));
    clearFQueue(&(dq->TIME));
}

// set all inputs to outputs to turn off lights on quit
void cleanQuit()
{
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT; // GPIO12
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT; // GPIO13
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT; // GPIO05
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT; // GPIO06
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT; // GPIO22
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT; // GPIO23
    raspicam_wrapper_destroy(Camera);
    free(rawImg);
    freeDataQueues(&dQueues);
}

void INThandler(int sig)
{
    cleanQuit();
    exit(0);
}

int setSpdLeft(int spd)
{
    int time_taken = 0;

    if (spd < PWM_MIN)
        spd = 0;

    if (spd > state.left_lvl)
    {
        if (spd >= RIGHT_MAX)
            spd = RIGHT_MAX;

        if (state.left_lvl < PWM_MIN)
            state.left_lvl = PWM_MIN;

        while (state.left_lvl < spd)
        {
            state.left_lvl += PWM_5_PERC;
            io->pwm->DAT1 = state.left_lvl; // PWMA right
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }
    else if (spd < state.left_lvl)
    {
        if (state.left_lvl < PWM_MIN)
            state.left_lvl = PWM_MIN;

        while (state.left_lvl > spd)
        {
            state.left_lvl -= PWM_5_PERC;
            if (state.left_lvl < 0)
                state.left_lvl = 0;
            io->pwm->DAT1 = state.left_lvl; // PWMA right
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }

    return time_taken;
}

int setSpdRight(int spd)
{
    int time_taken = 0;

    if (spd < PWM_MIN)
        spd = 0;

    if (spd > state.right_lvl)
    {
        if (spd >= PWM_MAX)
            spd = RIGHT_MAX;

        if (state.right_lvl < PWM_MIN)
            state.right_lvl = PWM_MIN;

        while (state.right_lvl < spd)
        {
            state.right_lvl += PWM_5_PERC;
            io->pwm->DAT2 = leftLookup[state.right_lvl]; // PWMA left
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }
    else if (spd < state.right_lvl)
    {
        if (state.right_lvl < PWM_MIN)
            state.right_lvl = PWM_MIN;

        while (state.right_lvl > spd)
        {
            state.right_lvl -= PWM_5_PERC;

            if (state.right_lvl < 0)
                state.right_lvl = 0;

            io->pwm->DAT2 = leftLookup[state.right_lvl]; // PWMA left
            usleep(RAMP_TIME);
            time_taken += RAMP_TIME;
        }
    }

    return time_taken;
}

void printData(int mode)
{
    pthread_mutex_lock(&fileWriteLock);
    char filename[] = "./hw10m1data.txt";
    sprintf(filename, "./hw10m%ddata.txt", state.mode);
    FILE *fp = fopen(filename, "r");

    char *token;
    char currentline[500];
    char lastLine[500];
    float maxMin[12]; // maxAX, maxAY, maxAZ, minAX, minAY, minAZ, maxGX, maxGY, maxGZ, minGX, minGY, minGZ = 0;
    float rangeAX, intAX, rangeAY, intAY, rangeAZ, intAZ, rangeGX, intGX, rangeGY, intGY, rangeGZ, intGZ, minAX, minAY, minAZ, minGX, minGY, minGZ = 0;

    while (fgets(currentline, sizeof(currentline), fp) != NULL)
    {
        strcpy(lastLine, currentline);
    }

    // get min max data
    token = strtok(lastLine, ",");
    for (int i = 0; i < 12; i++)
    {
        if (token == NULL)
            break;
        maxMin[i] = atof(token);
        token = strtok(NULL, ",");
    }

    minAX = maxMin[3];
    minAY = maxMin[4];
    minAZ = maxMin[5];
    minGX = maxMin[9];
    minGY = maxMin[10];
    minGZ = maxMin[11];

    rangeAX = maxMin[0] - minAX;
    rangeAY = maxMin[1] - minAY;
    rangeAZ = maxMin[2] - minAZ;
    rangeGX = maxMin[6] - minGX;
    rangeGY = maxMin[7] - minGY;
    rangeGZ = maxMin[8] - minGZ;

    intAX = rangeAX / 10;
    intAY = rangeAY / 10;
    intAZ = rangeAZ / 10;
    intGX = rangeGX / 10;
    intGY = rangeGY / 10;
    intGZ = rangeGZ / 10;

    fseek(fp, 0, SEEK_SET);

    printf("\n");

    float printVal;
    while (fgets(currentline, sizeof(currentline), fp) != NULL)
    {
        token = strtok(currentline, ",");
        if (token[0] == 'E')
            break;
        for (int i = 0; i < 6; i++)
        {
            switch (i)
            {
            case 0:
                printf("%d", (int)((atof(token) - minAX) / intAX));
                break;
            case 1:
                printf("%d", (int)((atof(token) - minAY) / intAY));
                break;
            case 2:
                printf("%d", (int)((atof(token) - minAZ) / intAZ));
                break;
            case 3:
                printf("%d", (int)((atof(token) - minGX) / intGX));
                break;
            case 4:
                printf("%d", (int)((atof(token) - minGY) / intGY));
                break;
            case 5:
                printf("%d ", (int)((atof(token) - minGZ) / intGZ));
                break;
            }
            token = strtok(NULL, ",");
        }
        printf("\n");
    }

    printf("\033[0;33mhw10>\033[0m ");

    fclose(fp);
    pthread_mutex_unlock(&fileWriteLock);
}

void printN()
{
    printf("\nxVel: %f, yVel %f\n", xVelocity, yVelocity); // FIXME
    printf("xPos: %f, yPos %f\n", xPos, yPos);             // FIXME
    printf("head: %f", heading);                           // FIXME
    printf("\n\033[0;33mhw10>\033[0m ");
}

void alarmHandler()
{
    if (!state.stopCollection)
        pthread_cond_broadcast(&dataCond);
}

void *getTimedPic()
{
    while (1)
    {
        if (state.end)
            break;

        pthread_mutex_lock(&alarmLock1);
        pthread_cond_wait(&dataCond, &alarmLock1);
        pthread_mutex_unlock(&alarmLock1);

        if (state.alarmCounter > 3) // ensures there is a new pic roughly every 100ms
        {
            raspicam_wrapper_grab(Camera);

            state.newPic = 1;
            state.alarmCounter = 0;
        }
        else
            state.alarmCounter++;
    }

    pthread_exit(0);
}

void *getTimedData()
{
    while (1)
    {
        if (state.end)
            break;

        pthread_mutex_lock(&alarmLock2);
        pthread_cond_wait(&dataCond, &alarmLock2);
        pthread_mutex_unlock(&alarmLock2);

        read_accelerometer_gyroscope(&calibration_accelerometer, &calibration_gyroscope, &state.sd, io->bsc);
        state.readCounts += 1;
    }
}

void dataToPPM(unsigned char *data, char *name, int width, int height, int size) // for testing
{
    FILE *outFile = fopen(name, "wb");
    if (outFile != NULL)
    {
        fprintf(outFile, "P6\n"); // write .ppm file header
        fprintf(outFile, "%d %d 255\n", width, height);
        // write the image data
        fwrite(data, 1, size, outFile);
        fclose(outFile);
        printf("Image, picture saved as %s\n", name);
    }
}

void arrToPPM(char *name, int width, int height, unsigned char data[width][height])
{
    unsigned char lin[height * width * 3];
    struct RGB_pixel *pixel = (struct RGB_pixel *)lin; // view data as R-byte, G-byte, and B-byte per pixel

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            pixel[width * i + j].R = data[j][i];
            pixel[width * i + j].G = data[j][i];
            pixel[width * i + j].B = data[j][i];
        }
    }

    dataToPPM(lin, name, width, height, width * height * 3);
}

unsigned char avgArr(int wStart, int hStart, int scale, int width, int height, unsigned char data[width][height])
{
    unsigned int avg = 0;

    wStart = wStart * scale;
    hStart = hStart * scale;

    for (int i = wStart; i < wStart + scale; i++)
    {
        for (int j = hStart; j < hStart + scale; j++)
        {
            avg += (unsigned int)data[i][j];
        }
    }

    return (unsigned char)(avg / (scale * scale));
}

int *threshold(int width, int height) // classify each pixel as over or under threshold and find center mass of laser
{
    int threshold = 230; // 90% threshold
    int xAvg = 0;
    int xCount = 0;
    int yAvg = 0;
    int yCount = 0;

    int avg[2];

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (state.image[j][i] >= threshold)
            {
                state.image[j][i] = 255;
                xAvg += j;
                yAvg += i;
                xCount++;
                yCount++;
            }
            else
                state.image[j][i] = 0;
        }
    }
    if (xCount > 0)
        avg[0] = xAvg / xCount;
    else
        avg[0] = 0;
    if (yCount > 0)
        avg[1] = yAvg / yCount;
    else
        avg[1] = 0;

    if (avg[0] < IMG_RED_WIDTH)
        state.xDist = avg[0] - IMG_RED_WIDTH / 2;
    else
        state.xDist = 0;
    if (avg[1] < IMG_RED_HEIGHT && avg[1] > 0)
        state.yDist = avg[1] - IMG_RED_HEIGHT / 2;
    else
        state.yDist = 0;
}

void sendToArray(unsigned char *data, int width, int height, bool print)
{
    unsigned char largeArr[width][height];
    struct RGB_pixel *pixel = (struct RGB_pixel *)data; // view data as R-byte, G-byte, and B-byte per pixel

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            largeArr[j][i] = pixel[width * i + j].R;
        }
    }

    uint32_t histogram[256];
    memset((void *)histogram, 0, 256 * sizeof(uint32_t));

    unsigned char pix;
    unsigned char maxPix = 0;
    unsigned char minPix = 0;
    for (int i = 0; i < IMG_RED_WIDTH; i++)
    {
        for (int j = 0; j < IMG_RED_HEIGHT; j++)
        {
            pix = avgArr(i, j, 4, width, height, largeArr);
            state.image[i][j] = pix;
        }
    }

    threshold(IMG_RED_WIDTH, IMG_RED_HEIGHT);

    if (print)
        arrToPPM("testarr.ppm", IMG_RED_WIDTH, IMG_RED_HEIGHT, state.image); // FIXME
}

void makeRedScale(unsigned char *data, unsigned int pixel_count)
{
    struct RGB_pixel *pixel;
    unsigned int pixel_index;
    unsigned char pixel_value;

    pixel = (struct RGB_pixel *)data; // view data as R-byte, G-byte, and B-byte per pixel

    for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
    {
        // gray scale => average of R color, G color, and B color intensity
        pixel[pixel_index].G = 0;
        pixel[pixel_index].B = 0;
    }
}

void processPic(bool printFull, bool printThresh)
{
    size_t image_size = raspicam_wrapper_getImageTypeSize(Camera, RASPICAM_WRAPPER_FORMAT_RGB);

    raspicam_wrapper_retrieve(Camera, rawImg, RASPICAM_WRAPPER_FORMAT_RGB);
    unsigned int pixHeight = raspicam_wrapper_getHeight(Camera);
    unsigned int pixWidth = raspicam_wrapper_getWidth(Camera);
    unsigned int pixel_count = pixHeight * pixWidth;

    makeRedScale(rawImg, pixel_count);
    sendToArray(rawImg, pixWidth, pixHeight, printThresh);

    if (printFull)
        dataToPPM(rawImg, "test1.ppm", pixWidth, pixHeight, image_size); // FIXME
}

void *dataAnalyze()
{

    pthread_mutex_lock(&dataCollectLock);
    pthread_cond_wait(&willCollectData, &dataCollectLock);
    pthread_mutex_unlock(&dataCollectLock);

    float AX, AY, AZ, GX, GY, GZ, RT;
    float maxAX = 0, maxAY = 0, maxAZ = 0, minAX = 0, minAY = 0, minAZ = 0, maxGX = 0, maxGY = 0, maxGZ = 0, minGX = 0, minGY = 0, minGZ = 0;

    pthread_mutex_lock(&fileWriteLock);
    char filename[] = "./hw10m1data.txt";
    sprintf(filename, "./hw10m%ddata.txt", state.mode);
    FILE *fp = fopen(filename, "w+");

    while (true)
    {
        if (state.end)
        {
            if (maxAX != 0 && minAY != 0)
                fprintf(fp, "EXTR\n%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", maxAX, maxAY, maxAZ, minAX, minAY, minAZ, maxGX, maxGY, maxGZ, minGX, minGY, minGZ);
            fclose(fp);
            pthread_mutex_unlock(&fileWriteLock);

            break;
        }
        if (dQueues.ACCEL_X.size == 0)
        {
            if (state.stopCollection)
            {
                if (maxAX != 0 && minAY != 0)
                    fprintf(fp, "EXTR\n%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", maxAX, maxAY, maxAZ, minAX, minAY, minAZ, maxGX, maxGY, maxGZ, minGX, minGY, minGZ);
                fclose(fp);

                pthread_mutex_unlock(&fileWriteLock);

                pthread_mutex_lock(&dataCollectLock);
                pthread_cond_wait(&willCollectData, &dataCollectLock);
                pthread_mutex_unlock(&dataCollectLock);

                if (state.end)
                    break;

                maxAX = 0, maxAY = 0, maxAZ = 0, minAX = 0, minAY = 0, minAZ = 0, maxGX = 0, maxGY = 0, maxGZ = 0, minGX = 0, minGY = 0, minGZ = 0;

                xVelocity = 0, yVelocity = 0; // m/s
                xPos = 0, yPos = 0;           // m
                heading = 0;

                sprintf(filename, "./hw10m%ddata.txt", state.mode);
                pthread_mutex_lock(&fileWriteLock);
                fp = fopen(filename, "w+");
            }
            else
                usleep(8);
        }
        else
        {
            AX = fQueuePop(&(dQueues.ACCEL_X));
            AY = fQueuePop(&(dQueues.ACCEL_Y));
            AZ = fQueuePop(&(dQueues.ACCEL_Z));
            GX = fQueuePop(&(dQueues.GYRO_X));
            GY = fQueuePop(&(dQueues.GYRO_Y));
            GZ = fQueuePop(&(dQueues.GYRO_Z));
            RT = fQueuePop(&(dQueues.TIME)) / CLOCKS_PER_SEC;

            maxAX = AX > maxAX ? AX : maxAX;
            maxAY = AY > maxAY ? AY : maxAY;
            maxAZ = AZ > maxAZ ? AZ : maxAZ;
            maxGX = GX > maxGX ? GX : maxGX;
            maxGY = GY > maxGY ? GY : maxGY;
            maxGZ = GZ > maxGZ ? GZ : maxGZ;
            maxAX = AX > maxAX ? AX : maxAX;
            maxAY = AY > maxAY ? AY : maxAY;
            maxAZ = AZ > maxAZ ? AZ : maxAZ;
            maxGX = GX > maxGX ? GX : maxGX;
            maxGY = GY > maxGY ? GY : maxGY;
            maxGZ = GZ > maxGZ ? GZ : maxGZ;
            minAX = AX < minAX ? AX : minAX;
            minAY = AY < minAY ? AY : minAY;
            minAZ = AZ < minAZ ? AZ : minAZ;
            minGX = GX < minGX ? GX : minGX;
            minGY = GY < minGY ? GY : minGY;
            minGZ = GZ < minGZ ? GZ : minGZ;
            minAX = AX < minAX ? AX : minAX;
            minAY = AY < minAY ? AY : minAY;
            minAZ = AZ < minAZ ? AZ : minAZ;
            minGX = GX < minGX ? GX : minGX;
            minGY = GY < minGY ? GY : minGY;
            minGZ = GZ < minGZ ? GZ : minGZ;

            heading = (heading + ((int)(GZ * 10) / 10) * RT);
            // if (GZ > 1)
            // printf("head: %d", ((int)GZ));

            AY = (((int)(AY * 10)) / 10.0);

            yVelocity += RT * (AY * cos(heading / 180 * PI) + AX * sin(heading / 180 * PI));
            yPos += RT * yVelocity;

            AX = (((int)(AX * 1)) / 1.0);

            xVelocity += RT * (AX * cos(heading / 180 * PI) + AY * sin(heading / 180 * PI));
            xPos += RT * xVelocity;

            fprintf(fp, "%f, %f, %f, %f, %f, %f \n", AX, AY, AZ, GX, GY, GZ);
        }
    }
    pthread_exit(0);
}

void *procPic()
{

    long remainingTime = 0, startLoop = 0, stopLoop = 0;
    pthread_mutex_lock(&takePicLock);
    pthread_cond_wait(&willTakePic, &takePicLock);
    pthread_mutex_unlock(&takePicLock);

    while (true)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;
        if (state.end)
        {
            break;
        }
        if (state.stopPics)
        { // stop collection if s is hit
            pthread_mutex_lock(&takePicLock);
            pthread_cond_wait(&willTakePic, &takePicLock);
            pthread_mutex_unlock(&takePicLock);
        }

        if (state.newPic)
        {
            state.newPic = 0;

            pthread_mutex_lock(&printLock);
            bool print = state.printPic;
            processPic(print, print);
            state.printPic = false;
            pthread_mutex_unlock(&printLock);
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PIC_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); // sleep for remaining portion of 100ms
    }
    pthread_exit(0);
}

void *dataCollect()
{
    long remainingTime = 0, startLoop = 0, stopLoop = 0;

    pthread_mutex_lock(&dataCollectLock);
    pthread_cond_wait(&willCollectData, &dataCollectLock);
    pthread_mutex_unlock(&dataCollectLock);

    ualarm(THREAD_PROCESS_TIME_uS, THREAD_PROCESS_TIME_uS); // set alarm
    state.readCounts = 0;

    while (true)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;
        if (state.end)
        {
            ualarm(0, 0); // reset alarm
            pthread_cond_broadcast(&dataCond);
            break;
        }
        if (state.stopCollection)
        {                 // stop collection if s is hit
            ualarm(0, 0); // reset alarm
            pthread_mutex_lock(&dataCollectLock);
            pthread_cond_wait(&willCollectData, &dataCollectLock);
            pthread_mutex_unlock(&dataCollectLock);

            if (state.end)
            {
                pthread_cond_broadcast(&dataCond);
                break;
            }

            state.readCounts = 0;
            ualarm(THREAD_PROCESS_TIME_uS, THREAD_PROCESS_TIME_uS); // set alarm
        }

        fQueuePush(THREAD_PROCESS_TIME_uS * state.readCounts, &(dQueues.TIME));

        state.readCounts = 0;

        fQueuePush((state.sd.ACCEL_XOUT.signed_value * calibration_accelerometer.scale - calibration_accelerometer.offset_x) * 9.81, &(dQueues.ACCEL_X));
        fQueuePush((state.sd.ACCEL_YOUT.signed_value * calibration_accelerometer.scale - calibration_accelerometer.offset_y) * 9.81, &(dQueues.ACCEL_Y));
        fQueuePush((state.sd.ACCEL_ZOUT.signed_value * calibration_accelerometer.scale - calibration_accelerometer.offset_z) * 9.81, &(dQueues.ACCEL_Z));

        fQueuePush(state.sd.GYRO_XOUT.signed_value * calibration_gyroscope.scale - calibration_gyroscope.offset_x, &(dQueues.GYRO_X));
        fQueuePush(state.sd.GYRO_YOUT.signed_value * calibration_gyroscope.scale - calibration_gyroscope.offset_y, &(dQueues.GYRO_Y));
        fQueuePush(state.sd.GYRO_ZOUT.signed_value * calibration_gyroscope.scale - calibration_gyroscope.offset_z, &(dQueues.GYRO_Z));

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); // sleep for remaining portion of 10ms
    }
    pthread_exit(0);
}

void *scheduler()
{
    initStack(&state.rightHist, HIST_QUEUE_SIZE);
    initStack(&state.leftHist, HIST_QUEUE_SIZE);

    int rHistp[HIST_QUEUE_SIZE];
    memset(rHistp, 0, sizeof(int) * HIST_QUEUE_SIZE);
    int lHistp[HIST_QUEUE_SIZE];
    memset(lHistp, 0, sizeof(int) * HIST_QUEUE_SIZE);

    state.rightHist.contents = rHistp;
    state.leftHist.contents = lHistp;

    long remainingTime = 0, startLoop = 0, stopLoop = 0, startTurn = 0, stopTurn = 0, m0TimerStart = 0, m0TimerStop = 0;
    uint32_t irRightVal;
    uint32_t irLeftVal;
    bool m2IsDriving = false;
    bool m1IsDriving = false;
    bool isReplaying = false;

    while (1)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;
        if (state.end)
        {
            pthread_cond_broadcast(&willTakePic);
            pthread_cond_broadcast(&willCollectData);
            pthread_cond_broadcast(&dataCond);
            break;
        }

        char command;

        m0TimerStop = clock() / CLOCKS_PER_MIRCO;
        if (state.mode == 0 && m0TimerStop - m0TimerStart > 5000000)
        {
            state.mode = 1;
            state.stopPics = true;
            state.stopCollection = true;
        }
        else if ((command = queuePop(&state.inputQueue)) != '\0')
        {
            if (command == 'm')
            {
                while ((command = queuePop(&state.inputQueue)) == '\0')
                    ;

                if (command == '1')
                {
                    // calibrate_accelerometer_and_gyroscope(&calibration_accelerometer, &calibration_gyroscope, io->bsc);
                    m1IsDriving = false;
                    m2IsDriving = false;
                    isReplaying = false;

                    state.stopCollection = false;
                    state.mode = 1;
                    pthread_mutex_lock(&rightControlLock);
                    clearQueue(&state.rightQueue);
                    queuePush('s', &state.rightQueue);
                    pthread_mutex_unlock(&rightControlLock);
                    pthread_mutex_lock(&leftControlLock);
                    clearQueue(&state.leftQueue);
                    queuePush('s', &state.leftQueue);
                    pthread_mutex_unlock(&leftControlLock);
                }
                else if (command == '2')
                {
                    // calibrate_accelerometer_and_gyroscope(&calibration_accelerometer, &calibration_gyroscope, io->bsc);
                    m1IsDriving = false;
                    m2IsDriving = false;
                    isReplaying = false;

                    state.stopCollection = false;
                    state.mode = 2;

                    pthread_mutex_lock(&rightControlLock);
                    clearQueue(&state.rightQueue);
                    queuePush('s', &state.rightQueue);
                    pthread_mutex_unlock(&rightControlLock);
                    pthread_mutex_lock(&leftControlLock);
                    clearQueue(&state.leftQueue);
                    queuePush('s', &state.leftQueue);
                    pthread_mutex_unlock(&leftControlLock);

                    GPIO_SET(io->gpio, 5); // set to forward
                    GPIO_CLR(io->gpio, 6);
                    GPIO_SET(io->gpio, 22); // set to forward
                    GPIO_CLR(io->gpio, 23);
                }
                else
                {
                    printf("\n\033[0;33mhw10>\033[0m ");
                    printf("m%c is not a command", command);
                }
            }

            else if (state.mode == 1)
            {
                if (command == 's')
                {
                    m1IsDriving = false;
                    m2IsDriving = false;
                    isReplaying = false;

                    state.stopCollection = true;
                    state.mode = 1;
                    pthread_mutex_lock(&rightControlLock);
                    clearQueue(&state.rightQueue);
                    queuePush('s', &state.rightQueue);
                    pthread_mutex_unlock(&rightControlLock);
                    pthread_mutex_lock(&leftControlLock);
                    clearQueue(&state.leftQueue);
                    queuePush('s', &state.leftQueue);
                    pthread_mutex_unlock(&leftControlLock);
                }
                else if (command == 'w' && state.leftDirection != 'w' && state.rightDirection != 'w')
                {
                    m1IsDriving = true;

                    clearStack(&state.rightHist);
                    clearStack(&state.leftHist);

                    state.stopCollection = false;
                    pthread_cond_broadcast(&willCollectData);
                }
                else if (command == 'p')
                {
                    printData(1);
                }
                else if (command == 'n')
                {
                    printN();
                }
                else if (command == 'r')
                {
                    isReplaying = true;
                }

                pthread_mutex_lock(&leftControlLock);
                queuePush(command, &state.leftQueue);
                pthread_mutex_unlock(&leftControlLock);

                pthread_mutex_lock(&rightControlLock);
                queuePush(command, &state.rightQueue);
                pthread_mutex_unlock(&rightControlLock);
            }
        }

        if (state.mode == 1)
        {
            if (isReplaying && !m1IsDriving)
            {
                if (!isReplaying)
                {
                    isReplaying = true;

                    GPIO_SET(io->gpio, 5); // set to forward
                    GPIO_CLR(io->gpio, 6);
                    GPIO_SET(io->gpio, 22); // set to forward
                    GPIO_CLR(io->gpio, 23);

                    io->pwm->DAT1 = 0;
                    io->pwm->DAT2 = 0;
                }

                if (state.rightHist.head - 1 >= -1)
                {

                    int lSpd = stackPop(&state.leftHist);
                    int rSpd = stackPop(&state.rightHist);

                    if (lSpd < 0 || rSpd < 0)
                    {
                        io->pwm->DAT1 = 0;
                        io->pwm->DAT2 = 0;

                        GPIO_SET(io->gpio, 5); // set to forward
                        GPIO_CLR(io->gpio, 6);
                        GPIO_SET(io->gpio, 22); // set to forward
                        GPIO_CLR(io->gpio, 23);

                        lSpd = -1 * lSpd;
                        rSpd = -1 * rSpd;
                    }
                    else
                    {
                        io->pwm->DAT1 = 0;
                        io->pwm->DAT2 = 0;

                        GPIO_CLR(io->gpio, 5); // set to backward
                        GPIO_SET(io->gpio, 6);
                        GPIO_CLR(io->gpio, 22); // set to backward
                        GPIO_SET(io->gpio, 23);

                        // int tmp = rSpd;
                        // rSpd = lSpd;
                        // lSpd = tmp;
                    }

                    if (lSpd > 0 && lSpd < 1000)
                        io->pwm->DAT1 = leftLookup[(int)lSpd];
                    else
                        io->pwm->DAT1 = 0;

                    if (rSpd > 0 && rSpd < 1000)
                        io->pwm->DAT2 = (int)rSpd;
                    else
                        io->pwm->DAT2 = 0;

                    printf("%d\n", lSpd);
                }
                else
                {
                    isReplaying = false;
                    io->pwm->DAT1 = 0;
                    io->pwm->DAT2 = 0;
                }
            }
        }

        if (state.mode == 2)
        {
            if (command == 's')
            {
                m2IsDriving = false;
                m1IsDriving = false;

                isReplaying = false;
                state.stopCollection = true;
                state.stopPics = true;
                state.mode = 2;

                pthread_mutex_lock(&rightControlLock);
                clearQueue(&state.rightQueue);
                queuePush('s', &state.rightQueue);
                pthread_mutex_unlock(&rightControlLock);
                pthread_mutex_lock(&leftControlLock);
                clearQueue(&state.leftQueue);
                queuePush('s', &state.leftQueue);
                pthread_mutex_unlock(&leftControlLock);
            }
            else if ((command == 'r' || isReplaying) && !m2IsDriving)
            {
                if (!isReplaying)
                {
                    isReplaying = true;

                    GPIO_SET(io->gpio, 5); // set to forward
                    GPIO_CLR(io->gpio, 6);
                    GPIO_SET(io->gpio, 22); // set to forward
                    GPIO_CLR(io->gpio, 23);
                }

                if (state.rightHist.head - 1 >= -1)
                {

                    int lSpd = stackPop(&state.leftHist);
                    int rSpd = stackPop(&state.rightHist);

                    if (lSpd < 0 || rSpd < 0)
                    {
                        io->pwm->DAT1 = 0;
                        io->pwm->DAT2 = 0;

                        GPIO_SET(io->gpio, 5); // set to forward
                        GPIO_CLR(io->gpio, 6);
                        GPIO_SET(io->gpio, 22); // set to forward
                        GPIO_CLR(io->gpio, 23);

                        lSpd = -1 * lSpd;
                        rSpd = -1 * rSpd;
                    }
                    else
                    {
                        io->pwm->DAT1 = 0;
                        io->pwm->DAT2 = 0;

                        GPIO_CLR(io->gpio, 5); // set to backward
                        GPIO_SET(io->gpio, 6);
                        GPIO_CLR(io->gpio, 22); // set to backward
                        GPIO_SET(io->gpio, 23);

                        // int tmp = rSpd;
                        // rSpd = lSpd;
                        // lSpd = tmp;
                    }

                    if (lSpd > 0 && lSpd < 1000)
                        io->pwm->DAT1 = leftLookup[(int)lSpd];
                    else
                        io->pwm->DAT1 = 0;

                    if (rSpd > 0 && rSpd < 1000)
                        io->pwm->DAT2 = (int)rSpd;
                    else
                        io->pwm->DAT2 = 0;

                    printf("%d\n", lSpd);
                }
                else
                {
                    isReplaying = false;
                    io->pwm->DAT1 = 0;
                    io->pwm->DAT2 = 0;
                }
            }
            else if (command == 'p')
            {
                printData(2);
            }
            else if (command == 'n')
            {
                printN();
            }
            else if (command == 'w' || m2IsDriving)
            {
                m2IsDriving = true;
                state.stopCollection = false;
                state.stopPics = false;
                pthread_cond_broadcast(&willTakePic);
                pthread_cond_broadcast(&willCollectData);

                if (command == 'w')
                {
                    clearStack(&state.rightHist);
                    clearStack(&state.leftHist);

                    GPIO_SET(io->gpio, 5); // set to forward
                    GPIO_CLR(io->gpio, 6);
                    GPIO_SET(io->gpio, 22); // set to forward
                    GPIO_CLR(io->gpio, 23);
                }

                int rSpd;
                int lSpd;
                float xPerc = (float)abs(state.xDist) / ((float)IMG_RED_WIDTH / 2.0); // percentage of half img
                float yPerc = (float)abs(state.yDist) / ((float)IMG_RED_HEIGHT / 2.0);

                if (state.yDist >= (IMG_RED_HEIGHT / 10))
                {
                    io->pwm->DAT1 = 0;
                    io->pwm->DAT2 = 0;
                    GPIO_SET(io->gpio, 5); // set to forward
                    GPIO_CLR(io->gpio, 6);
                    GPIO_SET(io->gpio, 22); // set to forward
                    GPIO_CLR(io->gpio, 23);

                    rSpd = (yPerc * ((PWM_RANGE - PWM_5_PERC * 2) / 2)) + PWM_MIN + PWM_5_PERC * 2;
                    lSpd = (yPerc * ((PWM_RANGE - PWM_5_PERC * 2) / 2)) + PWM_MIN + PWM_5_PERC * 2;

                    if (state.xDist < 0)
                    {
                        rSpd += xPerc * (PWM_RANGE / 2);
                        lSpd -= xPerc * (PWM_RANGE / 2);
                    }
                    else
                    {
                        lSpd += xPerc * (PWM_RANGE / 2);
                        rSpd -= xPerc * (PWM_RANGE / 2);
                    }

                    io->pwm->DAT1 = leftLookup[lSpd];
                    io->pwm->DAT2 = rSpd;
                }
                else if (state.yDist <= -1 * (IMG_RED_HEIGHT / 10))
                {
                    io->pwm->DAT1 = 0;
                    io->pwm->DAT2 = 0;
                    GPIO_CLR(io->gpio, 5); // set to backward
                    GPIO_SET(io->gpio, 6);
                    GPIO_CLR(io->gpio, 22); // set to backward
                    GPIO_SET(io->gpio, 23);

                    rSpd = PWM_MIN + PWM_5_PERC * 2;
                    lSpd = PWM_MIN + PWM_5_PERC * 2;

                    io->pwm->DAT1 = leftLookup[lSpd];
                    io->pwm->DAT2 = rSpd;

                    rSpd = (PWM_MIN + PWM_5_PERC * 2) * -1;
                    lSpd = (PWM_MIN + PWM_5_PERC * 2) * -1;
                }
                else
                {
                    rSpd = 0;
                    lSpd = 0;
                    io->pwm->DAT1 = lSpd;
                    io->pwm->DAT2 = rSpd;
                }

                if (lSpd > 1000)
                    lSpd = 1000;
                if (rSpd > 1000)
                    rSpd = 1000;

                stackPush(&state.rightHist, rSpd);
                stackPush(&state.leftHist, lSpd);
            }
        }

        if (m1IsDriving)
        {
            int8_t lScal = state.leftDirection == 'x' ? -1 : 1;
            int8_t rScal = state.rightDirection == 'x' ? -1 : 1;

            stackPush(&state.leftHist, state.left_lvl * lScal);
            stackPush(&state.rightHist, state.right_lvl * rScal);
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); // sleep for remaining portion of 10ms
    }
    pthread_exit(0);
}

// controls left motors
void *leftCtrl()
{

    char nextCommand;
    long remainingTime, startLoop, stopLoop;
    while (1)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;

        if (state.end)
        {
            break;
        }

        pthread_mutex_lock(&rightControlLock);
        nextCommand = queuePop(&state.leftQueue);
        pthread_mutex_unlock(&rightControlLock);

        if (nextCommand == 'i')
        {
            if (state.left_lvl >= RIGHT_MAX)
                state.left_lvl = RIGHT_MAX;
            else if (state.left_lvl == 0)
                state.left_lvl = PWM_MIN;
            else
                state.left_lvl += PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT1 = state.left_lvl; // PWMB right
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'j')
        {
            if (state.left_lvl <= PWM_MIN)
                state.left_lvl = 0;
            else
                state.left_lvl -= PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT1 = state.left_lvl; // PWMB right
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'w')
        {
            int tmpLvl = state.left_lvl;
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 22); // set to forward
                GPIO_CLR(io->gpio, 23);

                state.left_lvl = tmpLvl / 2;
                io->pwm->DAT1 = state.left_lvl; // PWMB right
                usleep(100000);                 // sleep .1 s
                state.left_lvl = tmpLvl;
                io->pwm->DAT1 = state.left_lvl; // PWMB right
            }
            if (state.leftDirection == 'x')
            {
                state.left_lvl = state.left_lvl / 2;
                io->pwm->DAT1 = state.left_lvl;
                ;               // PWMB right
                usleep(100000); // sleep .1 s

                state.left_lvl = 0;             // stop
                io->pwm->DAT1 = state.left_lvl; // PWMB right
                usleep(100000);                 // sleep .1s

                GPIO_SET(io->gpio, 22); // set to forward
                GPIO_CLR(io->gpio, 23);

                state.left_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT1 = state.left_lvl; // PWMB right
                usleep(100000);                 // sleep .1s
                state.left_lvl = tmpLvl;        // half speed
                io->pwm->DAT1 = state.left_lvl;
                ; // PWMB right
            }
            else
            {
                GPIO_SET(io->gpio, 22); // set to forward
                GPIO_CLR(io->gpio, 23);
            }
            state.leftDirection = 'w';
        }
        else if (nextCommand == 'x')
        {
            int tmpLvl = state.left_lvl;
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 23); // set to reverse
                GPIO_CLR(io->gpio, 22);

                state.left_lvl = state.left_lvl / 2;
                io->pwm->DAT1 = state.left_lvl;
                ;               // PWMB right
                usleep(100000); // sleep .1 s
                state.left_lvl = tmpLvl;
                io->pwm->DAT1 = state.left_lvl;
                ; // PWMB right
            }
            if (state.leftDirection == 'w')
            {
                state.left_lvl = state.left_lvl / 2;
                io->pwm->DAT1 = state.left_lvl; // PWMB right
                usleep(100000);                 // sleep .1 s

                state.left_lvl = 0;             // stop
                io->pwm->DAT1 = state.left_lvl; // PWMB right
                usleep(100000);                 // sleep .1s

                GPIO_SET(io->gpio, 23); // set to reverse
                GPIO_CLR(io->gpio, 22);

                state.left_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT1 = state.left_lvl; // PWMB right
                usleep(100000);                 // sleep .1s
                state.left_lvl = tmpLvl;        // half speed
                io->pwm->DAT1 = state.left_lvl; // PWMB right
            }
            else
            {
                GPIO_SET(io->gpio, 23); // set to reverse
                GPIO_CLR(io->gpio, 22);
            }
            state.leftDirection = 'x';
        }
        else if (nextCommand == 's')
        {
            state.left_lvl = state.left_lvl / 2;
            io->pwm->DAT1 = state.left_lvl; // PWMB right
            usleep(100000);                 // sleep .1 s
            state.left_lvl = 0;             // stop
            io->pwm->DAT1 = state.left_lvl; // PWMB right

            GPIO_CLR(io->gpio, 23); // set to stop
            GPIO_CLR(io->gpio, 22);
            state.leftDirection = 's';
        }
        else if (nextCommand == 'c')
        {
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 22); // set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.left_lvl;

            setSpdLeft(0);

            usleep(Z_C_TURN_TIME);

            setSpdLeft(tmpLvl);

            if (state.leftDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); // set to stop
                GPIO_CLR(io->gpio, 23);
            }
        }
        else if (nextCommand == 'z')
        {
            int turn_time = Z_C_TURN_TIME;
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 22); // set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.left_lvl;

            setSpdLeft(PWM_MAX);

            usleep(turn_time);

            setSpdLeft(tmpLvl);

            if (state.leftDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); // set to stop
                GPIO_CLR(io->gpio, 23);
            }
        }
        else if (nextCommand == 'd')
        {
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 22); // set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.left_lvl;
            state.left_lvl = PWM_MIN + 3 * PWM_5_PERC;
            io->pwm->DAT1 = state.left_lvl; // PWMB right
            usleep(TURN_TIME);
            state.left_lvl = tmpLvl;
            io->pwm->DAT1 = state.left_lvl; // PWMB right
            if (state.leftDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); // set to stop
                GPIO_CLR(io->gpio, 23);
            }
        }
        else if (nextCommand == 'a')
        {
            if (state.leftDirection == 's')
            {
                GPIO_SET(io->gpio, 22); // set to forward
                GPIO_CLR(io->gpio, 23);
            }
            int tmpLvl = state.left_lvl;
            state.left_lvl = RIGHT_MAX;
            io->pwm->DAT1 = state.left_lvl; // PWMB right
            usleep(TURN_TIME);
            state.left_lvl = tmpLvl;
            io->pwm->DAT1 = state.left_lvl; // PWMB right
            if (state.leftDirection == 's')
            {
                GPIO_CLR(io->gpio, 22); // set to stop
                GPIO_CLR(io->gpio, 23);
            }
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); // sleep for remaining portion of 10ms
    }

    pthread_exit(0);
}

void *rightCtrl()
{

    char nextCommand;
    long remainingTime, startLoop, stopLoop;
    while (1)
    {
        startLoop = clock() / CLOCKS_PER_MIRCO;

        if (state.end)
        {
            break;
        }

        pthread_mutex_lock(&leftControlLock);
        nextCommand = queuePop(&state.rightQueue);
        pthread_mutex_unlock(&leftControlLock);

        if (nextCommand == 'i')
        {
            if (state.right_lvl >= RIGHT_MAX)
                state.right_lvl = RIGHT_MAX;
            else if (state.right_lvl == 0)
                state.right_lvl = PWM_MIN;
            else
                state.right_lvl += PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT2 = leftLookup[state.right_lvl]; // PWMB left
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'j')
        {
            if (state.right_lvl <= PWM_MIN)
                state.right_lvl = 0;
            else
                state.right_lvl -= PWM_5_PERC;

            pthread_mutex_lock(&pwmLock);
            io->pwm->DAT2 = leftLookup[state.right_lvl]; // PWMB left
            pthread_mutex_unlock(&pwmLock);
        }
        else if (nextCommand == 'w')
        {
            int tmpLvl = state.right_lvl;
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 5); // set to forward
                GPIO_CLR(io->gpio, 6);

                state.right_lvl = tmpLvl / 2;
                io->pwm->DAT2 = state.right_lvl; // PWMB left
                usleep(100000);                  // sleep .1 s
                state.right_lvl = tmpLvl;
                io->pwm->DAT2 = state.right_lvl; // PWMB left
            }
            if (state.rightDirection == 'x')
            {
                state.right_lvl = state.right_lvl / 2;
                io->pwm->DAT2 = state.right_lvl;
                ;               // PWMB left
                usleep(100000); // sleep .1 s

                state.right_lvl = 0;             // stop
                io->pwm->DAT2 = state.right_lvl; // PWMB left
                usleep(100000);                  // sleep .1s

                GPIO_SET(io->gpio, 5); // set to forward
                GPIO_CLR(io->gpio, 6);

                state.right_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT2 = state.right_lvl; // PWMB left
                usleep(100000);                  // sleep .1s
                state.right_lvl = tmpLvl;        // half speed
                io->pwm->DAT2 = state.right_lvl;
                ; // PWMB left
            }
            else
            {
                GPIO_SET(io->gpio, 5); // set to forward
                GPIO_CLR(io->gpio, 6);
            }
            state.rightDirection = 'w';
        }
        else if (nextCommand == 'x')
        {
            int tmpLvl = state.right_lvl;
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 6); // set to reverse
                GPIO_CLR(io->gpio, 5);

                state.right_lvl = state.right_lvl / 2;
                io->pwm->DAT2 = state.right_lvl;
                ;               // PWMB left
                usleep(100000); // sleep .1 s
                state.right_lvl = tmpLvl;
                io->pwm->DAT2 = state.right_lvl;
                ; // PWMB left
            }
            if (state.rightDirection == 'w')
            {
                state.right_lvl = state.right_lvl / 2;
                io->pwm->DAT2 = state.right_lvl; // PWMB left
                usleep(100000);                  // sleep .1 s

                state.right_lvl = 0;             // stop
                io->pwm->DAT2 = state.right_lvl; // PWMB left
                usleep(100000);                  // sleep .1s

                GPIO_SET(io->gpio, 6); // set to reverse
                GPIO_CLR(io->gpio, 5);

                state.right_lvl = tmpLvl / 2;    // half speed
                io->pwm->DAT2 = state.right_lvl; // PWMB left
                usleep(100000);                  // sleep .1s
                state.right_lvl = tmpLvl;        // half speed
                io->pwm->DAT2 = state.right_lvl; // PWMB left
            }
            else
            {
                GPIO_SET(io->gpio, 6); // set to reverse
                GPIO_CLR(io->gpio, 5);
            }
            state.rightDirection = 'x';
        }
        else if (nextCommand == 's')
        {
            state.right_lvl = state.right_lvl / 2;
            io->pwm->DAT2 = state.right_lvl; // PWMB left
            usleep(100000);                  // sleep .1 s
            state.right_lvl = 0;             // stop
            io->pwm->DAT2 = state.right_lvl;
            ; // PWMB left

            GPIO_CLR(io->gpio, 6); // set to stop
            GPIO_CLR(io->gpio, 5);
            state.rightDirection = 's';
        }
        else if (nextCommand == 'z')
        {
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 5); // set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.right_lvl;

            setSpdRight(0);
            usleep(Z_C_TURN_TIME);
            setSpdRight(tmpLvl);

            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); // set to stop
                GPIO_CLR(io->gpio, 6);
            }
        }
        else if (nextCommand == 'c')
        {
            int turn_time = Z_C_TURN_TIME;

            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 5); // set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.right_lvl;

            setSpdRight(PWM_MAX);

            usleep(turn_time);

            setSpdRight(tmpLvl);
            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); // set to stop
                GPIO_CLR(io->gpio, 6);
            }
        }
        else if (nextCommand == 'a')
        {
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 5); // set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.right_lvl;
            state.right_lvl = leftLookup[PWM_MIN];
            io->pwm->DAT2 = state.right_lvl; // PWMB right
            usleep(TURN_TIME);
            state.right_lvl = tmpLvl;
            io->pwm->DAT2 = state.right_lvl; // PWMB right
            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); // set to stop
                GPIO_CLR(io->gpio, 6);
            }
        }
        else if (nextCommand == 'd')
        {
            if (state.rightDirection == 's')
            {
                GPIO_SET(io->gpio, 5); // set to forward
                GPIO_CLR(io->gpio, 6);
            }
            int tmpLvl = state.right_lvl;
            state.right_lvl = PWM_MAX;       // adfsaf
            io->pwm->DAT2 = state.right_lvl; // PWMB right
            usleep(TURN_TIME);
            state.right_lvl = tmpLvl;
            io->pwm->DAT2 = state.right_lvl; // PWMB right
            if (state.rightDirection == 's')
            {
                GPIO_CLR(io->gpio, 5); // set to stop
                GPIO_CLR(io->gpio, 6);
            }
        }

        stopLoop = clock() / CLOCKS_PER_MIRCO;
        remainingTime = THREAD_PROCESS_TIME_uS - stopLoop + startLoop;
        if (remainingTime > 0)
            usleep(remainingTime); // sleep for remaining portion of 10ms
    }
    pthread_exit(0);
}

// thread fuction to check for input
void *checkInput()
{
    char tempInput;
    while (1)
    {
        while ((tempInput = (char)getchar()) == '\0') // while input is not empty do nothing
            ;

        if (tempInput == 'r' || tempInput == 't' || tempInput == 'n' || tempInput == 'p' || tempInput == 's' || tempInput == 'w' || tempInput == 'x' || tempInput == 'i' || tempInput == 'j' || tempInput == 'a' || tempInput == 'd' || tempInput == 'q' || tempInput == 'm' || tempInput == '1' || tempInput == '2' || tempInput == '0' || tempInput == 'z' || tempInput == 'c')
        {
            if (tempInput == 'q')
            {
                state.end = true;
                break;
            }
            if (tempInput == 'm')
            {
                queuePush(tempInput, &state.inputQueue);

                while ((tempInput = (char)getchar()) == '\0') // while input is not empty do nothing
                    ;
            }

            queuePush(tempInput, &state.inputQueue);
        }

        printf("\n\033[0;33mhw10>\033[0m "); // newline for reading
    }

    pthread_exit(0);
}

void memset64(void *dest, uint64_t value, uintptr_t size)
{
    uintptr_t i;
    for (i = 0; i < (size & (~7)); i += 8)
    {
        memcpy(((char *)dest) + i, &value, 8);
    }
    for (; i < size; i++)
    {
        ((char *)dest)[i] = ((char *)&value)[i & 7];
    }
}

int main(void)
{
    signal(SIGINT, INThandler);
    signal(SIGALRM, alarmHandler);

    io = import_registers();
    Camera = raspicam_wrapper_create();
    init_gyro(io, &calibration_accelerometer, &calibration_gyroscope, &calibration_magnetometer);

    if (raspicam_wrapper_open(Camera))
    {
        printf("opening camera");

        sleep(1);

        raspicam_wrapper_grab(Camera);
        size_t image_size = raspicam_wrapper_getImageTypeSize(Camera, RASPICAM_WRAPPER_FORMAT_RGB);
        rawImg = (unsigned char *)malloc(image_size);
    }
    else
        printf("error opening camera\n");

    if (io != NULL)
    {
        /* print where the I/O memory was actually mapped to */
        printf("mem at 0x%8.8X\n", (unsigned int)io);

        /* set the pin function to OUTPUT for GPIO */
        enable_pwm_clock(io->cm, io->pwm);
        pwm_init(io, PWM_MAX);

        io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
        io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

        io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT; // AI1
        io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT; // AI2
        io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT; // BI1
        io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT; // BI2

        GPIO_CLR(io->gpio, 5); // init to stop
        GPIO_CLR(io->gpio, 6);
        GPIO_CLR(io->gpio, 22);
        GPIO_CLR(io->gpio, 23);

        printf("hit 'ctl c' or 'q' to quit\n");
        printf("\033[0;33mhw10>\033[0m "); // newline for reading

        static struct termios attr;
        tcgetattr(STDIN_FILENO, &attr);
        attr.c_lflag &= ~(ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &attr);

        struct publicState cArgs;

        state.end = false;
        state.stopCollection = true;
        state.stopPics = true;
        state.left_lvl = 0;
        state.right_lvl = 0;
        state.alarmCounter = 0;
        state.rightDirection = 's';
        state.leftDirection = 's';

        leftLookup[260] = 297;
        leftLookup[297] = 365;
        leftLookup[334] = 439;
        leftLookup[371] = 455;
        leftLookup[408] = 506;
        leftLookup[445] = 564;
        leftLookup[482] = 601;
        leftLookup[519] = 624;
        leftLookup[556] = 675;
        leftLookup[593] = 712;
        leftLookup[630] = 753;
        leftLookup[667] = 800;
        leftLookup[704] = 884;
        leftLookup[741] = 867;
        leftLookup[778] = 890;
        leftLookup[815] = 913;
        leftLookup[852] = 957;
        leftLookup[889] = 980;
        leftLookup[926] = 1000;

        memset64(leftLookup + 260, (uint64_t)297, sizeof(uint64_t) * 37);
        memset64(leftLookup + 297, (uint64_t)365, sizeof(uint64_t) * (334 - 297));
        memset64(leftLookup + 334, (uint64_t)439, sizeof(uint64_t) * (371 - 334));
        memset64(leftLookup + 371, (uint64_t)455, sizeof(uint64_t) * (408 - 371));
        memset64(leftLookup + 408, (uint64_t)506, sizeof(uint64_t) * (445 - 408));
        memset64(leftLookup + 445, (uint64_t)564, sizeof(uint64_t) * (482 - 445));
        memset64(leftLookup + 482, (uint64_t)601, sizeof(uint64_t) * (519 - 482));
        memset64(leftLookup + 519, (uint64_t)624, sizeof(uint64_t) * (556 - 519));
        memset64(leftLookup + 556, (uint64_t)675, sizeof(uint64_t) * (593 - 556));
        memset64(leftLookup + 593, (uint64_t)712, sizeof(uint64_t) * (630 - 593));
        memset64(leftLookup + 630, (uint64_t)753, sizeof(uint64_t) * (667 - 630));
        memset64(leftLookup + 667, (uint64_t)800, sizeof(uint64_t) * (704 - 667));
        memset64(leftLookup + 704, (uint64_t)884, sizeof(uint64_t) * (741 - 704));
        memset64(leftLookup + 741, (uint64_t)867, sizeof(uint64_t) * (778 - 741));
        memset64(leftLookup + 778, (uint64_t)890, sizeof(uint64_t) * (815 - 778));
        memset64(leftLookup + 815, (uint64_t)913, sizeof(uint64_t) * (852 - 815));
        memset64(leftLookup + 852, (uint64_t)957, sizeof(uint64_t) * (889 - 852));
        memset64(leftLookup + 889, (uint64_t)980, sizeof(uint64_t) * (926 - 889));
        memset64(leftLookup + 926, (uint64_t)1000, sizeof(uint64_t) * (1000 - 926));

        char leftContents[INSTR_QUEUE_SIZE];
        memset(leftContents, 0, INSTR_QUEUE_SIZE * sizeof(char));
        char rightContents[INSTR_QUEUE_SIZE];
        memset(rightContents, 0, INSTR_QUEUE_SIZE * sizeof(char));
        char inputContents[INSTR_QUEUE_SIZE];
        memset(inputContents, 0, INSTR_QUEUE_SIZE * sizeof(char));

        state.leftQueue.contents = leftContents;
        state.rightQueue.contents = rightContents;
        state.inputQueue.contents = inputContents;
        state.mode = 1; // FIXME true

        initDataQueues(&dQueues);

        initQueue(&state.leftQueue, INSTR_QUEUE_SIZE);
        initQueue(&state.rightQueue, INSTR_QUEUE_SIZE);
        initQueue(&state.inputQueue, INSTR_QUEUE_SIZE);

        pthread_attr_t pAttr;
        pthread_attr_init(&pAttr);

        pthread_mutex_init(&scheduleLock, NULL);
        pthread_mutex_init(&inputLock, NULL);
        pthread_mutex_init(&leftControlLock, NULL);
        pthread_mutex_init(&rightControlLock, NULL);
        pthread_mutex_init(&pwmLock, NULL);
        pthread_mutex_init(&printLock, NULL);
        pthread_mutex_init(&takePicLock, NULL);
        pthread_mutex_init(&fileWriteLock, NULL);
        pthread_mutex_init(&initLock, NULL);
        pthread_mutex_init(&dataCollectLock, NULL);
        pthread_mutex_init(&newPicLock, NULL);
        pthread_mutex_init(&alarmLock1, NULL);
        pthread_mutex_init(&alarmLock2, NULL);

        pthread_t inputThread, scheduleThread, leftThread, rightThread, dataCThread, dataAThread, procPicThread, timedDataThread, timedPicThread;
        pthread_create(&inputThread, &pAttr, &checkInput, NULL);       // create thread for input polling
        pthread_create(&scheduleThread, &pAttr, &scheduler, NULL);     // create thread for menu handling
        pthread_create(&leftThread, &pAttr, &leftCtrl, NULL);          // create thread for orange blue pwm control
        pthread_create(&rightThread, &pAttr, &rightCtrl, NULL);        // create thread for orange blue pwm control
        pthread_create(&procPicThread, &pAttr, &procPic, NULL);        // create thread for processing img
        pthread_create(&dataCThread, &pAttr, &dataCollect, NULL);      // create thread data collection
        pthread_create(&dataAThread, &pAttr, &dataAnalyze, NULL);      // create thread data collection
        pthread_create(&timedDataThread, &pAttr, &getTimedData, NULL); // create thread data collection
        pthread_create(&timedPicThread, &pAttr, &getTimedPic, NULL);   // create thread data collection

        pthread_join(inputThread, NULL);
        pthread_join(scheduleThread, NULL);
        pthread_join(leftThread, NULL);
        pthread_join(rightThread, NULL);
        pthread_join(procPicThread, NULL);
        pthread_join(dataCThread, NULL);
        pthread_join(dataAThread, NULL);
        // pthread_join(timedDataThread, NULL);
        // pthread_join(timedPicThread, NULL);

        pthread_mutex_destroy(&inputLock);
        pthread_mutex_destroy(&scheduleLock);
        pthread_mutex_destroy(&leftControlLock);
        pthread_mutex_destroy(&rightControlLock);
        pthread_mutex_destroy(&printLock);
        pthread_mutex_destroy(&pwmLock);
        pthread_mutex_destroy(&takePicLock);
        pthread_mutex_destroy(&fileWriteLock);
        pthread_mutex_destroy(&initLock);
        pthread_mutex_destroy(&newPicLock);
        pthread_mutex_destroy(&dataCollectLock);
        pthread_mutex_destroy(&alarmLock1);
        pthread_mutex_destroy(&alarmLock2);

        cleanQuit(); // turn off all lights and quit
    }
}
