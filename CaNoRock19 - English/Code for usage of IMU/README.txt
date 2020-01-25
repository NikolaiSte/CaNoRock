final_code:
	This is the file for programming the IMU-card. This might be updated,
	see https://github.com/NikolaiSte/CaNoRock for the newest version.

student_rocket_digital_interface_box:
	The file for simulation of the interface-box onboard the rocket.ten.

Usage of code:
	The easiest way to ensure a proper setup to check if the board is working properly
	is to first connect the interfacebox and open the code to run it. Ensure that
	arduino has the correct port and board is selected (Port varies with each time and board is teensy LC)
	
	Then connect the IMU-board to the computer and open the code to run it. Ensure that the port selected
	in this instance of arduino is different to that of the arduino instance running the interface-box,
	and that the correct board is selected. (Board should be Teensy LC, but one of the boards
	Steinar will bring from UiO is a Teensy 3.2 instead of LC)
	
	To test and verify the setup before launch, the DEBUG variable has to be set to 1.
	This is because without DEBUG set to 1 there will be nothing printed to the serial port and you wont be
	able to verify that all the sensors are sending information that makes sense.	
	