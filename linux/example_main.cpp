// main function intended as example
#include <iostream>
#include "SimpleSerial.h"

int main() {
    
    // setup port and baudrate
    std::string port = "/dev/ttyACM0";
	typedef unsigned long DWORD;
	DWORD COM_BAUD_RATE = B9600;
    
    // instantiate
    SimpleSerial* Serial;
    Serial = new SimpleSerial(port.c_str(), COM_BAUD_RATE);

    // get startup message into variable example
    std::string incoming = Serial->ReadSerialPort(2, "json");
    std::cout << incoming << std::endl;



    // test with first char of string and pointer
    std::string test = "5";
    char *test_ptr = &test[0];
    bool is_sent1 = Serial->WriteSerialPort(test_ptr);
    // print return message from arduino
    std::cout << Serial->ReadSerialPort(2, "json") << std::endl;



    // in some cases its necessary to add delay
    //  for our use case we need to wait for a motor to move a turntable
    sleep(2);



    // test with negative degree and simple pointer
    const char* command = "-5";
    bool is_sent2 = Serial->WriteSerialPort(command);
    // print return message from arduino
    std::cout << Serial->ReadSerialPort(2, "json") << std::endl;

    // close
    Serial->CloseSerialPort();

    return 0;
}