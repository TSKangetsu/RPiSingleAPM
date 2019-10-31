#include "SingleAPM.hpp"

int main()
{
	std::cout << "Welcome" << "\n";
	int fd = pca9685Setup(65, 0x40, 450);
	clock_t start;
	clock_t end;
	start = clock();
	pca9685PWMWrite(fd, 0, 900, 0);
	pca9685PWMWrite(fd, 1, 1200, 0);
	pca9685PWMWrite(fd, 2, 2000, 0);
	pca9685PWMWrite(fd, 3, 2300, 0);
	end = clock();
	std::cout << (end - start) << "\n";
}