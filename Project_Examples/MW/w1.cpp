/*
 * g++ -o w1 -I../../Drivers -I/usr/include/libusb-1.0 w1.cpp -L/usr/local/lib -Wl,-R/usr/local/lib -lrt -lm -lwiringPi -lboost_thread
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>

#define USE_THREADS	1
#include <math.h>
#include <time.h>
#include "BrickPiTick.h"
#include <wiringPi.h>
#include "BrickPi.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <boost/thread.hpp>

volatile int sensorTarget = 0;
boost::mutex mtx;

int motor1 = MOTOR_PORT_C;
int motor2 = MOTOR_PORT_D;
int motor3 = MOTOR_PORT_B;
int speed = 60;
int runTime = 60;

struct timeval start_time = { 0, 0 };

void handle_SIGINT(int unused) {
  // On CTRL+C - abort! //

  motorBankReset(motor1);
  motorBankReset(motor3);

  printf("\nBye!\n");
  exit(0);
}

int readline(int fd, char *buf, int len) {
  int available;
  int base = 0;
  int i;

  while (base < len - 1) {
    while ((available = serialDataAvail(fd)) == 0) {
      usleep(50000);
    }

    if (available < 0) {
      return available;
    }

    for (i = 0; i < available && base + i < len - 1; i++) {
      buf[base + i] = serialGetchar(fd);

      if (buf[base + i] == '\n') {
        buf[base + i - 1] = '\0';
        return base + i - 1;
      }
    }

    base += i;
  }
}

int isRunning() {
  mtx.lock();
  unsigned char s1 = motorGetStatusByte(motor1);
  unsigned char s2 = motorGetStatusByte(motor2);
  mtx.unlock();

  if ((s2 | s1) & MOTOR_STATUS_MOVING) {
    return 1;
  }

  return 0;
}

void waitToStop(unsigned int waitAfterwards) {
  while (isRunning()) {
    usleep(50000);
  }

  usleep(waitAfterwards);
}

char *getRelativeTime(char *buf, int buf_len) {
    struct timeval t, r;

    gettimeofday(&t, NULL);

    timersub(&t, &start_time, &r);

    snprintf(buf, buf_len, "%3u.%.3u", r.tv_sec, r.tv_usec / 1000);

    return buf;
}

void arduinoRun() {
  char line[256];
  char buf[256];
  int arduinoFd = serialOpen("/dev/ttyACM0", 115200);
  int start = 0;

  if (arduinoFd < 0) {
    return;
  }

  sensorTarget = 0;
  printf("%s: Calibrating!\n", getRelativeTime(buf, 256));
  serialPuts(arduinoFd, "C\n");
  sleep(12);
  sensorTarget = 255;
  printf("Done!\n");

  while (readline(arduinoFd, line, 256) >= 0) {
    if (strlen(line) > 0) {
      int arduinoValue = (int)strtol(line, (char **)NULL, 10);

      printf("%s: arduinoRun() line= %.128s  arduinoValue= %d  sensorTarget= %d\n",
             getRelativeTime(buf, 256), line, arduinoValue, sensorTarget);

      if (start && arduinoValue != 255 && (arduinoValue & sensorTarget) != sensorTarget) {
        mtx.lock();
        motorStop(motor1 | motor2, 1);
        usleep(150000);
        sensorTarget = arduinoValue;
        mtx.unlock();
        printf("%s: arduinoValue= %d\n", getRelativeTime(buf, 256), arduinoValue);
      }
      else {
        if (strstr(line, "C: ") != NULL) {
          start = 1;
          sensorTarget = arduinoValue = 255;
        }
      }
    }
  }
}

void avoid() {
  /*        ^
  *     4   |   2
  *    8+-------+  1
  *     |       |
  *     |       |
  *     |       |
  *     |D     C|
  *   16+-------+128
  *    32       64
  * 
  */ 
  int mySensors = sensorTarget;
  int l, r;
  char buf[256];

  l = 500 + (rand() % 500);
  r = 500 + (rand() % 500);

  printf("%s: avoid() sensorTarget= %d\n", getRelativeTime(buf, 256), mySensors);

  if ((mySensors & 0b11110000) == 0b11110000) {
    printf("         Front\n");

    sensorTarget = 0b11110000;
    motorRunTachometer(motor1 | motor2, 0, -speed, 720, 1, 1, 1);
    sensorTarget = 255;
    waitToStop(100000);

    if ((mySensors & 0b11111100) == 0b11111100) {
      printf("%s:     Rechts %d:%d  %d:%d\n", getRelativeTime(buf, 256), -speed, l, speed, r);

      motorRunTachometer(motor1, 0, speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, -speed, l, 1, 1, 1);
      waitToStop(100000);
    }
    else if ((mySensors & 0b11110011) == 0b11110011) {
      printf("%s:     Links %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

      motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
      waitToStop(100000);
    }
    else {
      l = r = 964;

      printf("%s:     Beide %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

      motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
      waitToStop(100000);
    }
  }
  else if ((mySensors & 0b00001111) == 0x00001111) {
    printf("          Back\n");

    sensorTarget = 0b00001111;
    motorRunTachometer(motor1 | motor2, 0, speed, 720, 1, 1, 1);
    sensorTarget = 255;
    waitToStop(100000);

    if ((mySensors & 0b00111111) == 0b00111111) {
      printf("%s:     Rechts %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

      motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
      waitToStop(100000);
    }
    else if ((mySensors & 0b11001111) == 0b11001111) {
      printf("%s:     Links %d:%d  %d:%d\n", getRelativeTime(buf, 256), -speed, l, speed, r);

      motorRunTachometer(motor1, 0, speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, -speed, l, 1, 1, 1);
      waitToStop(100000);
    }
    else {
      l = r = 964;

      printf("%s:     Beide %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

      motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
      waitToStop(100000);
    }
  }
  else {
    printf("          Strange!\n");

    motorStop(motor1 | motor2, 1);
    sensorTarget = 255;
    waitToStop(100000);
  }
}

int main(int argc, char *argv[]) {
  int result;
  char buf[256];

  // Catch CTRL+C (SIGINT) signals //
  signal(SIGHUP, handle_SIGINT);
  signal(SIGINT, handle_SIGINT);
  signal(SIGQUIT, handle_SIGINT);
  signal(SIGTERM, handle_SIGINT);

  srand(time(NULL));
  gettimeofday(&start_time, NULL);

  ClearTick();

  result = BrickPiSetup();

  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;

  result = motorBankReset(motor1);
  result = motorBankReset(motor3);

  result = BrickPiSetupSensors();

  boost::thread thread_1 = boost::thread(arduinoRun);

  sleep(15);
  printf("%s: main start!\n", getRelativeTime(buf, 256));

  time_t startTime = time(NULL);

  while (startTime + runTime > time(NULL)) {
    if (isRunning() == 0) {
      if (sensorTarget != 255) {
        avoid();
      }
      else {
        motorRunUnlimited(motor1 | motor2, 0, speed);
      }
    }
    else {
      usleep(200000);
    }
  }

  printf("%s: End!!\n", getRelativeTime(buf, 256));

  if (isRunning() != 0) {
    motorStop(motor1 | motor2, 1);
  }

  usleep(150000);

  result = motorBankReset(motor1);
  result = motorBankReset(motor3);
}
