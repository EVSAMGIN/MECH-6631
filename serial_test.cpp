#include <iostream>
#include <Windows.h>

using namespace std;

int main()
{
	HANDLE hSerial = CreateFile("\\\\.\\COM7", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hSerial == INVALID_HANDLE_VALUE) {
		cout << "Error opening COM7. Error code: " << GetLastError() << "\n";
		cout << "Check: Device Manager > Ports to verify COM port number\n";
		return 1;
	}

	PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(dcb);
	GetCommState(hSerial, &dcb);
	dcb.BaudRate = 115200;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;
	SetCommState(hSerial, &dcb);

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	SetCommTimeouts(hSerial, &timeouts);

	cout << "COM7 opened. Sending signals...\n";

	double signal = 0.0;
	char buffer[50];
	char readBuffer[256];
	DWORD bytesWritten, bytesRead;

	while (true) {
		// Send signal
		sprintf(buffer, "%.2f\n", signal);
		WriteFile(hSerial, buffer, strlen(buffer), &bytesWritten, NULL);
		cout << "Sent: " << buffer;

		// Read response
		if (ReadFile(hSerial, readBuffer, sizeof(readBuffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
			readBuffer[bytesRead] = '\0';
			cout << "Received: " << readBuffer;
		}

		Sleep(1000);
		signal += 10.0;
		if (signal > 180.0) signal = -180.0;
	}

	CloseHandle(hSerial);
	return 0;
}
