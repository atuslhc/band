/*****************************************************************************
 * The fall detect module will calculate the accelerometer 3-axis data to
 * energy curve and report the result. The caller can gather all of them and 
 * issue a signal. In the module do not support the timer to keep or ignore
 * the result in a certain duration. User can use a external variable and timer
 * to do it.
 * Generally, the sensor have a buffer(FIFO) store the data, or user can store'
 * the sampling data by external array while reading each sampling. We just need 
 * call the module per second to keep the data process up to date. The caller 
 * should call Init_Fall_Detect() after system initialize, then parsing the
 * data to module by array to Calc_axis_rms() to prepare the first stage data.
 * If the system already have a function calculated the 3-axis RMS value, he can
 * skip the Calc_axis_rms() and call the Fall_Detect(), and then call
 * post_Fall_Detect() adjust external axis_rms[] index.
 *****************************************************************************/

The sensor running is continuous, so call the Fall_Detect() cyclic, and we
SHOULD put a external variable to keep the flag in a device task. For example,

void DeviceTask()
{
	case MEMS_INT1_Message:
		if (systemStatus.memsSensorOn) {
			Calc_Fall_Detect(&MEMS_BUFF[0][0], &MEMS_BUFF[1][0], &MEMS_BUFF[2][0]);
			FD_result += Fall_Detect(axis_rms, len);
		}
		break;
}

We can perform a check function in other thread/task keep watch the result (FD_result)
and broadcast the flag in the ble broadcast user define bytes continue 5 minutes.

void onRealTimeClockTick()
{
	....
#if FALL_DETECT_SUPPORT
	CheckFall();
#endif
#if SOS_HIT_SUPPORT
	CheckSOS();
#endif
	SendNotificationAlert();
	PermitUpdateBroadcast();
	...
}

void CheckFall(void)
{
	extern uint8_t FD_result; //initialize 0
	static bool fallDetectedTimeCount = true;
	static bool fallFlashOledTimeCount = true;
	static time_t startTime = 0;
	static time_t startTimeFlashOled = 0;
	time_t fiveMinutes = 0;
	time_t flashOledanyTime = 0;
	extern uint8_t oldFallStatus; //[BG033] static change to global and move to mems_track.h
	uint8_t currentFallStatus = FD_result;

	if(oldFallStatus != currentFallStatus)
	{ // get a falling
		oldFallStatus = currentFallStatus;
		isDetectedFall = true;
		SendAlertNotification = 1;

		/* set display flash duration 10 seconds */
		blAlertMenu = true;
		UnLockScreen(false);
		OLEDON(); // turn on oled display
		NoTOUCHPressCount = 10;//keep 10 seconds flash screeen w/o button press.

		ForceShowMenu(1, MENU_TYPE_FALL_ALERT);

		if(fallFlashOledTimeCount)
		{
			fallFlashOledTimeCount = false; //lock
			startTimeFlashOled = time(NULL); //get the start timestamp.
		}

		if(fallDetectedTimeCount)
		{
			fallDetectedTimeCount = false;
			startTime = time(NULL);
		}
	}

	if(isDetectedFall == true) //get the fall flag, start counting time.
	{
		fiveMinutes = time(NULL);
		flashOledanyTime = time(NULL);
	}

	if(flashOledanyTime - startTimeFlashOled >= 12)
	{
		fallFlashOledTimeCount = true; //clean flag for next trigger.
		blAlertMenu = false;

		//force jump to Time menu
		if(IsForcedShowMenu())
		{
			UnforceShowMenu();

			JumpToMenu(MENU_TYPE_Time);
		}
	}

	if(fiveMinutes - startTime >= FIVE_MINUTES_SECONDS)
	{ //over 5 minutes, clean the fall event.
		fallDetectedTimeCount = true; //clean timeflag for next trigger.
		isDetectedFall = false; //clean while 5 minutes, terminate
		SendAlertNotification = 0xFF;
	}
}
#endif
