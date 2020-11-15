#include <iostream>
#pragma once
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <asm-generic/termbits.h>

class Ibus
{
public:
	inline Ibus() { Ibus_fd = -1;/* This only for init*/ };
	// UartDevice is dev file , such as "/dev/ttyS0"
	inline Ibus(char* UartDevice)
	{
		Ibus_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
		struct termios2 options { };

		if (0 != ioctl(Ibus_fd, TCGETS2, &options)) {
			close(Ibus_fd);
			Ibus_fd = -1;
		}

		options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
			| IXON);
		options.c_iflag |= (INPCK | IGNPAR);
		options.c_oflag &= ~OPOST;
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		options.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
		options.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
		options.c_ispeed = 115200;
		options.c_ospeed = 115200;
		options.c_cc[VMIN] = 32;
		options.c_cc[VTIME] = 0;

		if (0 != ioctl(Ibus_fd, TCSETS2, &options)) {
			close(Ibus_fd);
			Ibus_fd = -1;
		}
	}
	/*int waitTime support > 2000 , microSeconds
	  @lose_HoldTime at lease 1 , if you set to zero , will loop forever untill sbus data comfirmly ready*/
	inline int IbusRead(int* channelsData, int waitTime, int lose_HoldTime)
	{
		if (Ibus_fd == -1)
			return -1;
		FD_ZERO(&fd_Maker);
		FD_SET(Ibus_fd, &fd_Maker);
		lose_frameCount = 0;
		while (true) {
			InputBuffer = read(Ibus_fd, &ibusData, sizeof(ibusData));
			if (InputBuffer == 32) {
				if (ibusData[0] == 0x20 && ibusData[1] == 0x40)
					break;
			}
			lose_frameCount += 1;
			if (lose_frameCount == lose_HoldTime)
				return -1;
			usleep(waitTime);
		}

		IbusParser(ibusData, ChannelsData);
		for (size_t i = 0; i < 16; i++)
		{
			channelsData[i] = (int)ChannelsData[i];
		}
		return lose_frameCount;
	}

	inline void IbusParser(uint8_t* IbusRaw, uint16_t* Channel)
	{
		for (size_t i = 0; i < 14; i++)
		{
			Channel[i] = IbusRaw[i * 2 + 1 + 2] * 255 + IbusRaw[i * 2 + 2];
		}
	}
private:
	int Ibus_fd;
	int InputBuffer;
	int lose_frameCount;
	fd_set fd_Maker;
	uint8_t ibusData[33];
	uint16_t ChannelsData[14];
};