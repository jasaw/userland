/*
 *    Filename: serial.c
 * Description: serial.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "serial.h"


#ifndef BSL_RST
#define BSL_RST     24
#endif

#ifndef BSL_TEST
#define BSL_TEST    21
#endif

#ifndef BSL_TCK
#define BSL_TCK     20
#endif


static int serial_fd = -1;


void serial_flush_input_output(int ttyfd)
{
    tcflush(ttyfd, TCIOFLUSH);
}


int set_non_block(int fd, unsigned char enable)
{
    int result = 0;
    int flags = fcntl(fd, F_GETFL);
    if (flags >= 0)
    {
        if (enable)
            flags |= O_NONBLOCK;
        else
            flags &= ~O_NONBLOCK;
        if (fcntl(fd, F_SETFL, flags) < 0)
            result = -1;
    }
    else
        result = -1;
    return result;
}


static int configure_serial_port(int baudrate)
{
    struct termios newtio;

    memset(&newtio, 0, sizeof(newtio));
    tcgetattr(serial_fd, &newtio);

    speed_t brate = B115200;

    switch (baudrate)
    {
        case 50:     brate = B50;     break;
        case 75:     brate = B75;     break;
        case 110:    brate = B110;    break;
        case 134:    brate = B134;    break;
        case 150:    brate = B150;    break;
        case 200:    brate = B200;    break;
        case 300:    brate = B300;    break;
        case 600:    brate = B600;    break;
        case 1200:   brate = B1200;   break;
        case 1800:   brate = B1800;   break;
        case 2400:   brate = B2400;   break;
        case 4800:   brate = B4800;   break;
        case 9600:   brate = B9600;   break;
        case 19200:  brate = B19200;  break;
        case 38400:  brate = B38400;  break;
        case 57600:  brate = B57600;  break;
        case 115200: brate = B115200; break;
        case 230400: brate = B230400; break;
        default: fprintf(stderr, "Error: invalid baudrate %d\n", baudrate); return -1;
    }

    cfsetospeed (&newtio, brate);
    cfsetispeed (&newtio, brate);

    /* Setting other Port Stuff */
    newtio.c_cflag     &=  ~PARENB;            // Make 8n1
    newtio.c_cflag     &=  ~CSTOPB;
    newtio.c_cflag     &=  ~CSIZE;
    newtio.c_cflag     |=  CS8;

    newtio.c_cflag     &=  ~CRTSCTS;           // no flow control
    newtio.c_cc[VMIN]   =  0;                  // read doesn't block
    newtio.c_cc[VTIME]  =  0;                  // 0.5 seconds read timeout
    newtio.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&newtio);

    tcsetattr(serial_fd, TCSANOW, &newtio);

    serial_flush_input_output(serial_fd);

    return 0;
}


void serial_close(void)
{
    if (serial_fd >= 0)
    {
        close(serial_fd);
        serial_fd = -1;
    }
}


int serial_init(const char *device_path, int baudrate)
{
    serial_close();
    serial_fd = open(device_path, O_RDWR | O_NONBLOCK | O_NDELAY);
    if (serial_fd < 0)
    {
        fprintf(stderr, "Error: unable to open %s\n", device_path);
        return -1;
    }
    if (configure_serial_port(baudrate) != 0)
    {
        fprintf(stderr, "Error: unable to configure %s\n", device_path);
        return -1;
    }

    return 0;
}


int serial_get_fd(void)
{
    return serial_fd;
}
