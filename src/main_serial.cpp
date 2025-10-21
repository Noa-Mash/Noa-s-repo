#include <stdio.h>
#include <signal.h>
#include "ceserial.h"
using namespace std;

volatile bool running = true;

//-----------------------------------------------------------
void signalHandler(int signum) {
    printf("\nReceived Ctrl+C, exiting...\n");
    running = false;
}

//-----------------------------------------------------------
int main()
{
    // Set up Ctrl+C handler
    signal(SIGINT, signalHandler);

#ifdef CE_WINDOWS
    ceSerial com("\\\\.\\COM10",38400,8,'N',1); // Windows
#else
    ceSerial com("/dev/ttyS0",38400,8,'N',1); // Linux
#endif

    printf("Opening port %s.\n",com.GetPort().c_str());
    if (com.Open() == 0)
    {
        printf("OK.\n");
    }
    else 
    {
        printf("Error.\n");
        return 1;
    }

    // Comment out the writes
    /*
    bool successFlag;
    printf("Writing.\n");
    char s[]="Hello";
    successFlag=com.Write(s); // write string
    successFlag=com.WriteChar('!'); // write a character
    
    printf("Waiting 3 seconds.\n");
    ceSerial::Delay(3000); // delay to wait for a character
    */

    printf("Reading data... Press Ctrl+C to exit.\n");
    
    // Continuous reading loop
    char buffer[256];
    while(running)
	{
        int bytesAvailable = com.Available();
        if (bytesAvailable > 0) 
		{
            int bytesRead = com.ReadBuffer(buffer, sizeof(buffer) - 1);
            if (bytesRead > 0) 
			{
                buffer[bytesRead] = '\0'; // null terminate
                printf("%s", buffer);
                fflush(stdout);
            }
        }
        ceSerial::Delay(10); // Small delay to prevent busy waiting
    }

    printf("Closing port %s.\n",com.GetPort().c_str());
    com.Close();
    return 0;
}
