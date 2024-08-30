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

#define VERSION "1.0"
#define AUTHOR "Mauro Soligo"
#define EMAIL "mauro.soligo@gmail.com"
#define DATE "August 29, 2024"
#define NUM_ITERATIONS 1000

void print_banner()
{
    printf("*******************************************\n");
    printf("* Real-Time Serial Test Program           *\n");
    printf("* Version: %-30s *\n", VERSION);
    printf("* Date: %-33s *\n", DATE);
    printf("* Author: %-31s *\n", AUTHOR);
    printf("* Contact: %-30s *\n", EMAIL);
    printf("*******************************************\n\n");
}

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
    } else {
        printf("Process set to real-time priority (SCHED_FIFO) with priority 99.\n");
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

void print_timestamped_message(const char *serial_port)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    printf("%15ld.%09ld: Character 0x80 sent on %s\n", ts.tv_sec, ts.tv_nsec, serial_port);
}

void measure_performance(int fd, const char *serial_port)
{
    struct timespec start, end, req, diff;
    long long min_jitter = 0, max_jitter = 0, total_jitter = 0;
    long long expected_interval_ns = 1000000L; // 1 millisecondo in nanosecondi

    req.tv_sec = 0;
    req.tv_nsec = expected_interval_ns;

    for (int i = 0; i < NUM_ITERATIONS; i++)
    {
        clock_gettime(CLOCK_REALTIME, &start);

        send_character(fd, 0x80);  // Per fronte di salita
        print_timestamped_message(serial_port);

        // Attendi il prossimo periodo
        if (nanosleep(&req, NULL) < 0)
        {
            perror("nanosleep failed");
            exit(EXIT_FAILURE);
        }

        clock_gettime(CLOCK_REALTIME, &end);

        // Calcola la differenza di tempo
        diff.tv_sec = end.tv_sec - start.tv_sec;
        diff.tv_nsec = end.tv_nsec - start.tv_nsec;
        if (diff.tv_nsec < 0) {
            diff.tv_sec -= 1;
            diff.tv_nsec += 1000000000L;
        }

        long long actual_interval_ns = diff.tv_sec * 1000000000L + diff.tv_nsec;
        long long jitter = actual_interval_ns - expected_interval_ns;

        if (i == 0) {
            min_jitter = max_jitter = jitter;
        } else {
            if (jitter < min_jitter) min_jitter = jitter;
            if (jitter > max_jitter) max_jitter = jitter;
        }
        total_jitter += jitter;
    }

    printf("\nPerformance Statistics:\n");
    printf("Minimum jitter: %lld ns\n", min_jitter);
    printf("Maximum jitter: %lld ns\n", max_jitter);
    printf("Average jitter: %lld ns\n", total_jitter / NUM_ITERATIONS);
}

int main(int argc, char *argv[])
{
    int fd;
    char *serial_port = NULL;
    int opt;

    print_banner();

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

    // Misura le performance dello scheduler RT
    measure_performance(fd, serial_port);

    close(fd);
    return 0;
}
