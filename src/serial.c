#include "darknet.h"
#include "serial.h"
#include <fcntl.h>
#include <unistd.h>

int fd = 0;
unsigned char map[81][8];

// 1 millisecond = 0.001 microseconds
unsigned int vest_interval = 250000; // 500 milliseconds

void open_port() {
	if (fd == 0)
	{
		fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);

		if (fd == 1)
			printf("\n  Error! in Opening %s\n", "/dev/ttyS0");
		else
			printf("\n %s Opened Successfully\n", "/dev/ttyS0");

		struct termios SerialPortSettings;	/* Create the structure                          */

		tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

		cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
		cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

		SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
		SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

		SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
		
		
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

		SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

		if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
			printf("\n  ERROR ! in Setting attributes");	
		else
			printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");

		usleep(2000000);
	}
}

void close_port() {
	if (fd == 1)
		close(fd);/* Close the Serial port */
}

void serialize_detections(image im, detection *dets, int num, float thresh, char **names, image **alphabet, int classes)
{
    int i,j;
    for(i = 0; i < num; ++i){
        char labelstr[4096] = {0};
        int class = -1;
        for(j = 0; j < classes; ++j){
            if (dets[i].prob[j] > thresh){
                if (class < 0) { // The class will be > 0 only if we have two similar probabilities for the same bbox
                    strcat(labelstr, names[j]);
                    class = j;
                    break; // <<<--- So adding this break because of the performance. We show only the first.
                } 
                // else {
                //     strcat(labelstr, ", ");
                //     strcat(labelstr, names[j]);
                // }
                printf("%s: %.0f%%\n", names[j], dets[i].prob[j]*100);
            }
        }
        if(class >= 0){
            box b = dets[i].bbox;

            int left  = (b.x-b.w/2.)*im.w;
            int right = (b.x+b.w/2.)*im.w;
            int top   = (b.y-b.h/2.)*im.h;
            int bot   = (b.y+b.h/2.)*im.h;

            if(left < 0) left = 0;
            if(right > im.w-1) right = im.w-1;
            if(top < 0) top = 0;
            if(bot > im.h-1) bot = im.h-1;

			int cols = 2;
			int rows = 4;
			float a[cols][rows];
			
			int i = 0;
			unsigned char position[cols * rows];
			for(size_t row = 0; row < rows; row++) {
				for(size_t col = 0; col < cols; col++) {
					float x1, y1, x2, y2;
					float q_x1, q_y1, q_x2, q_y2;

					q_x1 = (im.w / cols) * col;
					q_y1 = (im.h / rows) * row;
					q_x2 = (im.w / cols) * (col + 1);
					q_y2 = (im.h / rows) * (row + 1);

					if (q_x1 > left)
						x1 = q_x1;
					else
						x1 = left;
					
					if (q_y1 > top)
						y1 = q_y1;
					else
						y1 = top;

					if (q_x2 > right)
						x2 = right;
					else
						x2 = q_x2;

					if (q_y2 > bot)
						y2 = bot;
					else
						y2 = q_y2;
						
					// percent of the detected object in the quadrant
					a[col][row] = ((x2 - x1) * (y2 - y1)) /
								  ((right - left) * (bot - top));
					// pixel
					position[i++] = (unsigned char)(int)(a[col][row] * 255);
				}
			}
			
			size_t size = 12;
    		unsigned char *signal = malloc(size * sizeof(unsigned char));

			// Set motors values from map array
			unsigned char my_class[8];
			for(size_t i = 0; i < 8; i++) {
				my_class[i] = map[class+1][i];
			}
			getPattern(signal, my_class, sizeof(my_class));
            sendToSerialWithSleep(signal, size);
            getOff(signal);
            sendToSerialWithSleep(signal, size);

			// Set motors values from position of the object
			getPattern(signal, position, sizeof(position));
            sendToSerialWithSleep(signal, size);
            getOff(signal);
            sendToSerialWithSleep(signal, size);

        }
    }
}

// void getOn(unsigned char* out) {
// 	// 255 255 255
// 	// 255 255 255
// 	// 255 255 255
// 	unsigned char x[10] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
// 	getPattern(out, x, sizeof(x));
// }

void getOff(unsigned char* out) {
	// 0 0 0
	// 0 0 0
	// 0 0 0
	unsigned char x[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	getPattern(out, x, sizeof(x));
}

void fillPatternsMap() {
	unsigned char values[3] = {1, 127, 255};

	// number of loops = number of motors (or group of motors)
	
	for(size_t i = 0; i < 81; i++) {
		for(size_t n1 = 0; n1 < 3; n1++) {
			for(size_t n2 = 0; n2 < 3; n2++) {
				for(size_t n3 = 0; n3 < 3; n3++) {
					for(size_t n4 = 0; n4 < 3; n4++) {
						map[i][0] = values[n1];
						map[i][1] = values[n1];
						map[i][2] = values[n2];
						map[i][3] = values[n2];
						map[i][4] = values[n3];
						map[i][5] = values[n3];
						map[i][6] = values[n4];
						map[i][7] = values[n4];
						i++;
					}
				}
			}
		}
	}

}

void getPattern(unsigned char* out, unsigned char* values, size_t size) {
	out[0] = '<';
	for(int i=0; i < size; ++i){
    	out[i+1] = (values[i] == 0 ? 1 : values[i]);
  	}
	out[size+1] = '>';
	// out[size+2] = '\0';
}

void sendToSerialWithSleep(unsigned char* data, size_t size) {
	writeDataToSerial("/dev/ttyS0", data, size);
	printf("\n Waiting %d milliseconds", vest_interval);
	usleep(vest_interval);
}

void sendToSerial(unsigned char* data, size_t size) {
	writeDataToSerial("/dev/ttyS0", data, size);
}

/* The data cannot contain zero char values.
When 0x00, send 0x01 to avoid serial communication issues. */
int writeDataToSerial(char *port, unsigned char* data, size_t size) {
	
	/*------------------------------- Write data to serial port -----------------------------*/
	//char write_buffer[] = "<ÿÿÿÿÿÿÿÿÿÿ>";	/* Buffer containing characters to write into port	     */	
	//char write_buffer[] = "<>";
	int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port */ 

	bytes_written = write(fd, data, size);
	printf("\n  %s written to %s", data, port);
	printf("\n  %d Bytes written to %s", bytes_written, port);
	printf("\n +----------------------------------+\n\n");

	//return Status;
	return 1;
}