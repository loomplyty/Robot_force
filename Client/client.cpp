#include <Robot_Client.h>

using namespace std;

int main(int argc, char *argv[])
{
	Robots::SendRequest(argc, argv, "/usr/Robots/resource/HexapodIII/HexapodIII.xml");

	return 0;
}
