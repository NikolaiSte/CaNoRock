IMU_Readout_CaNoRockXVIII.m: Code for analysis of data
	This is the code used for the analysis of the data post-launch.
	This is just a readout from the files recieved from telemetry and reversing the
	encoding of the data that was done before it was sent from the rocket.

CaNoRock_XVIII_Launch.mat: File with all the readouts from all sensors
	Here it is generally Data1_D1_GPSIMU_Raw and GPSIMU_frame that is of interest.

	Data1_D1_GPSIMU_Raw is the raw data read out from the IMUs

	GPSIMU_frame show the structure of the sent and recieved data. 160 is the sync-word for a new subframe,
	161 is the sync-word for a new frame. 1 frame contains 5 subframes which in turn contains 20 bytes,
	making it a total of 100 bytes. Subframe 0 is information from GPS, subframes 1,2,3 is information
	from the LSM, BMI and ICM respectively.

Rocket_data_Easy_AlexJan.m: Code for readout and analysis of all the sensors onboard the rocket
	This file will probably not be that interesting, nbut I've added it here anyways. Here you can find
	readouts and analysis of all the data from all the other sensors on the CaNoRock18 launch. This is mostly
	used for post-launch analysis and comparison of analog sensors and their digital counterparts etc.