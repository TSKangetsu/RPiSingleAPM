#include "SingleAPM.hpp"

int main()
{
	std::cout << "Welcome" << "\n";
	int fd = pca9685Setup(300, 0x40, 480);
	clock_t start;
	clock_t end;
	start = clock();
	pca9685PWMWrite(fd, 4, 0, 2300);
	pca9685PWMWrite(fd, 5, 0, 2300);
	pca9685PWMWrite(fd, 6, 0, 2300);
	pca9685PWMWrite(fd, 7, 0, 2300);
	end = clock();
	for (int i = 2400; i < 4000; i++)
	{
		start = clock();
		pca9685PWMWrite(fd, 4, 0, i);
		pca9685PWMWrite(fd, 5, 0, i);
		pca9685PWMWrite(fd, 6, 0, i);
		pca9685PWMWrite(fd, 7, 0, i);
		end = clock();
		usleep(10000);
		std::cout << "action time: " << (end - start) << "\n" << "pwm : " << i << "\n";
	}
	start = clock();
	pca9685PWMWrite(fd, 4, 0, 3000);
	pca9685PWMWrite(fd, 5, 0, 3000);
	pca9685PWMWrite(fd, 6, 0, 3000);
	pca9685PWMWrite(fd, 7, 0, 3000);
	end = clock();
	std::cout << (end - start) << "\n";
	sleep(2);
	start = clock();
	pca9685PWMWrite(fd, 4, 0, 4000);
	pca9685PWMWrite(fd, 5, 0, 4000);
	pca9685PWMWrite(fd, 6, 0, 4000);
	pca9685PWMWrite(fd, 7, 0, 4000);
	end = clock();
	std::cout << (end - start) << "\n";
	sleep(2);
	pca9685PWMReset(fd);
}