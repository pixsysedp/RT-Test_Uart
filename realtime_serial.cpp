#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <sched.h>
#include <errno.h>
#include <getopt.h>

void print_usage(const char *prog_name)
{
    printf("Usage: %s -p <serial_port>\n", prog_name);
    printf("  -p <serial_port>  Serial port (e.g., /dev/ttyS0)\n");
}

void set_realtime_priority()
{
    struct sched_param schedParam;
    schedParam.sched_priority = 99; // Massima priorità per SCHED_FIFO
    if (sched_setscheduler(0, SCHED_FIFO, &schedParam) != 0)
    {
        perror("sched_setscheduler failed");
        exit(EXIT_FAILURE);
    }
}

int configure_serial_port(const char *port_name)
{
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("open failed");
        exit(EXIT_FAILURE);
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr failed");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // Configurazione della velocità della porta seriale a 1 Mbit/s
    cfsetospeed(&tty, B1000000);
    cfsetispeed(&tty, B1000000);

    // Configurazione 8N1 (8 bit di dati, nessuna parità, 1 bit di stop)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD); // Disabilita parità
    tty.c_cflag &= ~CSTOPB;            // 1 bit di stop
    tty.c_cflag &= ~CRTSCTS;           // Disabilita controllo hardware

    tty.c_iflag &= ~IGNBRK; // Non ignorare i break
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disabilita controllo di flusso software

    tty.c_lflag = 0; // Modalità raw, nessun echo
    tty.c_oflag = 0; // Modalità raw

    tty.c_cc[VMIN] = 1;  // Leggi almeno 1 carattere
    tty.c_cc[VTIME] = 1; // Timeout di lettura 0.1 secondi

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr failed");
        close(fd);
        exit(EXIT_FAILURE);
    }

    return fd;
}

void send_character(int fd, char c)
{
    if (write(fd, &c, 1) != 1)
    {
        perror("write failed");
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char *argv[])
{
    int fd;
    struct timespec req;
    char *serial_port = NULL;
    int opt;

    while ((opt = getopt(argc, argv, "p:h")) != -1)
    {
        switch (opt)
        {
        case 'p':
            serial_port = optarg;
            break;
        case 'h':
        default:
            print_usage(argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    if (serial_port == NULL)
    {
        print_usage(argv[0]);
        exit(EXIT_FAILURE);
    }

    // Set high priority for the process
    set_realtime_priority();

    // Configure the serial port
    fd = configure_serial_port(serial_port);
    printf("Serial port %s configured at 1Mbit/s\n", serial_port);

    // Set the periodic time interval to 1 ms
    req.tv_sec = 0;
    req.tv_nsec = 10000L; // 0.01 millisecond

    while (1)
    {
        //send_character(fd, 'A');
	send_character(fd, 0x80);
        printf("Character 'A' sent on %s\n", serial_port);

        // Wait for the next period
        if (nanosleep(&req, NULL) < 0)
        {
            perror("nanosleep failed");
            exit(EXIT_FAILURE);
        }
    }

    close(fd);
    return 0;
}
