/*
 * g++ -o cam '-D__LIBPIXY_VERSION__="0.2"' -I../../Drivers -I/usr/include/libusb-1.0 cam.cpp -L/usr/local/lib -Wl,-R/usr/local/lib -lpixyusb -lrt -lm -lwiringPi -lboost_thread
*/
#include <fcntl.h>
#include <math.h>
#include <memory.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>
#include <wiringPi.h>
#include <boost/thread.hpp>
#include <linux/i2c-dev.h>
#include <sys/time.h>
#include <X11/X.h>
#include <X11/Xlib.h>

#define USE_THREADS	1
#include "pixy.h"
#include "BrickPiTick.h"
#include "BrickPi.h"

char red[] = "#FF0000";

struct Block myBlock;
volatile bool foundBlock = false;
boost::mutex blockMtx;

volatile int sensorTarget = 0;
boost::mutex mtx;

int motor1 = MOTOR_PORT_C;
int motor2 = MOTOR_PORT_D;
int motor3 = MOTOR_PORT_B;
int speed = 60;

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
        usleep(250000);
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

#define BLOCK_BUFFER_SIZE  10

void pixyRun() {
  char buf[256];
  int pixy_init_status;
  struct Block blocks[BLOCK_BUFFER_SIZE];
  struct Block emptyBlock = { 0, 0, 0, 0, 0, 0, 0 };
  int blocks_copied;

  pixy_init_status = pixy_init();
  if(!pixy_init_status == 0) {			// Error initializing Pixy
    printf("pixy_init(): ");
    pixy_error(pixy_init_status);

    return;
  }

  while (1) {
    blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);


    if (blocks_copied <= 0) {
      blockMtx.lock();
      myBlock = emptyBlock;
      foundBlock = false;
      blockMtx.unlock();
    }
    else {
      struct Block *foundBlockPtr = &emptyBlock;

      for (int i = 0; i < blocks_copied; i++) {
        if (blocks[i].type == TYPE_NORMAL) {
          if (blocks[i].signature == 1) {
            unsigned int a;

            if ((a = blocks[i].width * blocks[i].height) >= 16 &&
                a > foundBlockPtr->width * foundBlockPtr->height) {
              foundBlockPtr = &(blocks[i]); 
            }
          }
        }
      }

      blockMtx.lock();
      myBlock = *foundBlockPtr;
      foundBlock = (foundBlockPtr != &emptyBlock);
      blockMtx.unlock();
    }

    usleep(200000);
  }
}

bool avoid(bool grab) {
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

  printf("%s: avoid(%d) sensorTarget= %d\n", getRelativeTime(buf, 256), grab, mySensors);

  if ((mySensors & 0b11110000) == 0b11110000) {
    printf("         Front\n");

    if (grab) {
      if ((mySensors & 0b11111100) == 0b11111100) {
        printf("%s:     turn right to drop\n", getRelativeTime(buf, 256));
        sensorTarget = 0b11111100;
        motorRunTachometer(motor2, 0, 45, 1440, 1, 1, 1);
        waitToStop(150000);
      }
      else if ((mySensors & 0b11110011) == 0b11110011) {
        printf("%s:     turn left to drop\n", getRelativeTime(buf, 256));
        sensorTarget = 0b11110011;
        motorRunTachometer(motor1, 0, 45, 1440, 1, 1, 1);
        waitToStop(150000);
      }
      else {
        printf("%s:     drop\n", getRelativeTime(buf, 256));
        motorRunTachometer(motor3, 0, 50, 80, 1, 1, 0);
        waitToStop(250000);

        sensorTarget = 0b11110000;
        motorRunTachometer(motor1 | motor2, 0, -speed, 720, 1, 1, 1);
        sensorTarget = 255;
        waitToStop(150000);

        motorRunTachometer(motor1, 0, -speed, 964, 1, 0, 1);
        motorRunTachometer(motor2, 0, speed, 964, 1, 1, 1);
        waitToStop(150000);

        return false;
      }
    }
    else {
      sensorTarget = 0b11110000;
      motorRunTachometer(motor1 | motor2, 0, -speed, 720, 1, 1, 1);
      sensorTarget = 255;
      waitToStop(150000);

      if ((mySensors & 0b11111100) == 0b11111100) {
        printf("%s:     Rechts %d:%d  %d:%d\n", getRelativeTime(buf, 256), -speed, l, speed, r);

        motorRunTachometer(motor1, 0, speed, r, 1, 0, 1);
        motorRunTachometer(motor2, 0, -speed, l, 1, 1, 1);
        waitToStop(150000);
      }
      else if ((mySensors & 0b11110011) == 0b11110011) {
        printf("%s:     Links %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

        motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
        motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
        waitToStop(150000);
      }
      else {
        l = r = 964;

        printf("%s:     Beide %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

        motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
        motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
        waitToStop(150000);
      }
    }
  }
  else if ((mySensors & 0b00001111) == 0x00001111) {
    printf("          Back\n");

    sensorTarget = 0b00001111;
    motorRunTachometer(motor1 | motor2, 0, speed, 720, 1, 1, 1);
    sensorTarget = 255;
    waitToStop(150000);

    if ((mySensors & 0b00111111) == 0b00111111) {
      printf("%s:     Rechts %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

      motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
      waitToStop(150000);
    }
    else if ((mySensors & 0b11001111) == 0b11001111) {
      printf("%s:     Links %d:%d  %d:%d\n", getRelativeTime(buf, 256), -speed, l, speed, r);

      motorRunTachometer(motor1, 0, speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, -speed, l, 1, 1, 1);
      waitToStop(150000);
    }
    else {
      l = r = 964;

      printf("%s:     Beide %d:%d  %d:%d\n", getRelativeTime(buf, 256), speed, l, -speed, r);

      motorRunTachometer(motor1, 0, -speed, r, 1, 0, 1);
      motorRunTachometer(motor2, 0, speed, l, 1, 1, 1);
      waitToStop(150000);
    }
  }
  else {
    printf("          Strange!\n");

    motorStop(motor1 | motor2, 1);
    sensorTarget = 255;
    waitToStop(150000);
  }

  return grab;
}

int main(int argc, char *argv[]) {
  int result;
  char buf[256];
  bool grab = false;
  struct Block blck;

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
  boost::thread thread_2 = boost::thread(pixyRun);

  Display* dis = XOpenDisplay(NULL);
  Window win = XCreateSimpleWindow(dis, RootWindow(dis, 0), 10, 10, 10, 10,
                                   0, BlackPixel (dis, 0), BlackPixel(dis, 0));

  Atom wm_state = XInternAtom(dis, "_NET_WM_STATE", False);
  Atom fullscreen = XInternAtom(dis, "_NET_WM_STATE_FULLSCREEN", False);

  XEvent xev;
  memset(&xev, 0, sizeof(xev));
  xev.type = ClientMessage;
  xev.xclient.window = win;
  xev.xclient.message_type = wm_state;
  xev.xclient.format = 32;
  xev.xclient.data.l[0] = 1;
  xev.xclient.data.l[1] = fullscreen;
  xev.xclient.data.l[2] = 0;

  XMapWindow(dis, win);
  XSendEvent (dis, DefaultRootWindow(dis), False,
              SubstructureRedirectMask | SubstructureNotifyMask, &xev);

  XFlush(dis);

  XColor red_col;
  Colormap colormap = DefaultColormap(dis, 0);
  GC red_gc = XCreateGC(dis, win, 0, 0);
  XParseColor(dis, colormap, red, &red_col);
  XAllocColor(dis, colormap, &red_col);
  XSetForeground(dis, red_gc, red_col.pixel);

  sleep(15);
  printf("%s: main start!\n", getRelativeTime(buf, 256));

  time_t startTime = time(NULL);

  while (startTime + 120 > time(NULL)) {
    int search_count = 0;

    while (startTime + 120 > time(NULL) && sensorTarget == 255 && !grab) {
      int x_x = blck.x - blck.width / 2;
      int x_y = blck.y - blck.height / 2;

      if (x_x > 0) {
        x_x--;
      }

      if (x_y > 0) {
        x_y--;
      }

      XClearArea(dis, win, x_x, x_y, blck.width + 2, blck.height + 2, false);

      if (!foundBlock) {
        if (search_count++ > 3) {
          XFlush(dis);
          printf("%s: search\n", getRelativeTime(buf, 256));
          motorRunTachometer(motor1, 0, 35, 60, 1, 0, 1);
          motorRunTachometer(motor2, 0, -35, 60, 1, 1, 1);
        }

        waitToStop(350000);
      }
      else {
	blockMtx.lock();
        blck = myBlock;
        blockMtx.unlock();
        search_count = 0;

        printf("%s: [sig:%2u w:%3u h:%3u x:%3u y:%3u]\n",
               getRelativeTime(buf, 256),
               blck.signature,
               blck.width,
               blck.height,
               blck.x,
               blck.y);

        x_x = blck.x - blck.width / 2;
        x_y = blck.y - blck.height / 2;
        XFillRectangle(dis, win, red_gc, x_x, x_y, blck.width, blck.height);
        XFlush(dis);

        int x = blck.x;
        int y = blck.y;

        if (y < 120) {
          if (x < 140) {
            printf("%s: turn left\n", getRelativeTime(buf, 256));
            motorRunTachometer(motor1, 0, 35, 30, 1, 0, 1);
            motorRunTachometer(motor2, 0, -35, 30, 1, 1, 1);
          }
          else if(x > 180) {
            printf("%s: turn right\n", getRelativeTime(buf, 256));
            motorRunTachometer(motor1, 0, -35, 30, 1, 0, 1);
            motorRunTachometer(motor2, 0, 35, 30, 1, 1, 1);
          }
          else {
            printf("%s: forward\n", getRelativeTime(buf, 256));
            motorRunTachometer(motor1 | motor2, 0, 35, 180 - y, 1, 1, 1);
          }
        }
        else {
          printf("%s: grab\n", getRelativeTime(buf, 256));
          motorRunTachometer(motor3, 0, -50, 80, 1, 1, 0);
          grab = true;
        }

        waitToStop(350000);
      }
    }

    if (isRunning() == 0) {
      if (sensorTarget != 255) {
        grab = avoid(grab);
      }
      else {
        motorRunUnlimited(motor1 | motor2, 0, speed);
      }
    }
    else {
      usleep(50000);
    }
  }

  printf("%s: End!!\n", getRelativeTime(buf, 256));

  if (grab) {
    motorRunTachometer(motor3, 0, 50, 80, 1, 1, 0);
  }

  if (isRunning() != 0) {
    motorStop(motor1 | motor2, 1);
  }

  usleep(150000);

  result = motorBankReset(motor1);
  result = motorBankReset(motor3);
}
