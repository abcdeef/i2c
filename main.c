#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <signal.h>
//asd12

#define M_PI 3.14159265358979323846
#define HMC5883L_I2C_ADDR 0x1E
#define MAX 2

void writeToDevice(int fd, int reg, int val);

short s_x[MAX], s_y[MAX], s_z[MAX];
int z_x = -1, z_y = -1, z_z = -1;
int fd;

void printBin(unsigned char *CH) {
    unsigned char Mask = 0x01;
    unsigned char P2_B2[8];
    int n = 7;
    do {
        P2_B2[n--] = *CH & Mask ? 1 : 0;
    } while (Mask <<= 1);
    for (n = 0; n < 8; n++)
        printf("%hho", P2_B2[n]);
    printf("\n\r");
}

void printBin2(unsigned short *CH) {
    unsigned short Mask = 1;
    unsigned char P2_B2[16];
    int n = 15;
    do {
        P2_B2[n--] = *CH & Mask ? 1 : 0;
    } while (Mask <<= 1);
    for (n = 0; n < 16; n++)
        printf("%hho", P2_B2[n]);
    printf("\n\r");
}

float avg(short *array) {
    float tmp = 0;
    for (int n = 0; n < MAX; n++) {
        tmp += *(array + n);
    }
    return (tmp / MAX);
}

void put(short *array, int *pointer, short *val) {
    (*pointer)++;
    if (*pointer > MAX - 1)
        *pointer = 0;

    *(array + *pointer) = *val;
}

void selectDevice(int fd, int addr, char * name) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr, "%s not present\n", name);
    }
}

void selftest(void) {
    writeToDevice(fd, 0x00, 0x71);
    writeToDevice(fd, 0x01, 0xA0);
    writeToDevice(fd, 0x02, 0x00);

    usleep(10000);
}

void writeToDevice(int fd, int reg, int val) {
    char buf[3];
    buf[0] = 0x3C;
    buf[1] = reg;
    buf[2] = val;

    if (write(fd, buf, 3) != 3) {
        fprintf(stderr, "Can't write to ADXL345\n");
    }
}

void shutdown(int sig) {
    printf("Exit\n");
    close(fd);
}

int main(int argc, char** argv) {
    setvbuf(stdout, (char *) NULL, _IONBF, 0);
    signal(SIGINT, shutdown);

    unsigned char buf[16];
    float heading;
    float declinationAngle = 3.5f * M_PI / 180.0f;

    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        fprintf(stderr, "Failed to open i2c bus\n");

        return 1;
    }


    // initialise ADXL345
    selectDevice(fd, HMC5883L_I2C_ADDR, "HMC5883L");

    writeToDevice(fd, 0x01, 32);
    writeToDevice(fd, 0x02, 0);

    short gx = 0, gy = 0, gz = 0;
    for (int i = 1; i < 10000; ++i) {
        buf[0] = 0x03;

        if ((write(fd, buf, 1)) != 1) {
            // Send the register to read from
            fprintf(stderr, "Error writing to i2c slave\n");
        }

        if (read(fd, buf, 6) != 6) {
            fprintf(stderr, "Unable to read from HMC5883L\n");
        } else {
            gx = (buf[0] << 8) | buf[1];
            gz = (buf[2] << 8) | buf[3];
            gy = (buf[4] << 8) | buf[5];

#define SENSORS_GAUSS_TO_MICROTESLA       (100)  
#define _hmc5883_Gauss_LSB_XY  1100.0
#define _hmc5883_Gauss_LSB_Z   980


            //printf("%u %hho %hho\n\r",gx,buf[0],buf[1]);
            /*put(s_x, &z_x, &gx);
            put(s_y, &z_y, &gy);
            put(s_z, &z_z, &gz);
             */

            //double ba = sqrt(pow(gx, 2) + pow(gy, 2) + pow(gz, 2));
            //double bb = sqrt(pow(N_X, 2) + pow(N_Y, 2) + pow(N_Z, 2));
            //angle = acos((gx * N_X + gy * N_Y + gz * N_Z) / ba / bb) *180.0f / M_PI;

            heading = atan2(gy / _hmc5883_Gauss_LSB_XY, gx / _hmc5883_Gauss_LSB_XY); // + declinationAngle;
            printf("%i %f\n\r", gx, heading * 180.0f / M_PI);

            /*
            if (heading < 0)
                heading += 2 * M_PI;

            if (heading > 2 * M_PI)
                heading -= 2 * M_PI;
             */
            //printf("x=%hi, y=%hi, z=%hi  angle = %0.1f\n\r", gx, gy, gz, heading * 180.0 / M_PI);
            //printf("x=%.2f, y=%.2f, z=%.2f  angle = %0.1f\n\r", avg(s_x), avg(s_y), avg(s_z), heading * 180.0 / M_PI);
        }

        usleep(300000);
    }


    return (EXIT_SUCCESS);
}
