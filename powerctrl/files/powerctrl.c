/*
* Copyright (C) 2013 - 2016  Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software,
* and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in this
* Software without prior written authorization from Xilinx.
*
*/

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>

#define GPIO_DATA_REG  0x0
#define GPIO_TRI_REG   0x4

#ifndef SENSOR_PATH
#define SENSOR_PATH "/sys/class/hwmon/hwmon0/temp1_input"
#endif

volatile int keep_running = 0;

int read_and_print_temp(void) {
    FILE *fp = fopen(SENSOR_PATH, "r");
    if (!fp) {
        perror("open sensor");
        return -1;
    }
    int temp_milli = 0;
    if (fscanf(fp, "%d", &temp_milli) == 1) {
        printf("Temperature: %.3f C\n", temp_milli / 1000.0);
    } else {
        perror("read sensor");
    }
    fclose(fp);
    fflush(stdout);
    return 0;
}

void *temp_loop(void *arg) {
    while (keep_running) {
        read_and_print_temp();
        sleep(1);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    const char *uio = (argc > 1) ? argv[1] : "/dev/uio3";

    int fd = open(uio, O_RDWR);
    if (fd < 0) {
        perror("open uio");
        return 1;
    }

    void *base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    volatile uint32_t *gpio_data = (uint32_t*)((char*)base + GPIO_DATA_REG);
    volatile uint32_t *gpio_tri  = (uint32_t*)((char*)base + GPIO_TRI_REG);
    *gpio_tri = 0x0;

    printf("AXI GPIO ready. (UIO: %s)\n", uio);
    printf("Enter 0=start, 1=stop, q=quit\n");
    fflush(stdout);

    pthread_t thread = 0;
    char cmd = 1;

    while (1) {
        if(cmd == '0')
            cmd = '1';
        else
            cmd = '0';

        if (cmd == '0') {
            if (!keep_running) {
                *gpio_data = 0;
                keep_running = 1;
                pthread_create(&thread, NULL, temp_loop, NULL);
                printf("Started temperature monitoring (1s interval)\n");
            } else {
                printf("Already running.\n");
            }
        } 
        else if (cmd == '1') {
            if (keep_running) {
                keep_running = 0;
                pthread_join(thread, NULL);
                *gpio_data = 1;
                printf("Stopped temperature monitoring\n");
            }
        } 
        else if (cmd == 'q' || cmd == 'Q') {
            if (keep_running) {
                keep_running = 0;
                pthread_join(thread, NULL);
            }
            break;
        } 
        else {
            printf("Unknown cmd. Use 1/0/q\n");
        }
        fflush(stdout);

        sleep(5);
    }

    munmap((void*)base, 0x1000);
    close(fd);
    return 0;
}
