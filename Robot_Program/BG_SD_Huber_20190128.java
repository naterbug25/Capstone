package Capstone_Huber;

import java.math.BigInteger;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import com.kuka.generated.ioAccess.AnybusIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting class.<br>
 * The cyclic background task can be terminated via {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or stopping of the task.
 * 
 * @see UseRoboticsAPIContext
 * 
 */
/*********************************************************************************************
 * Step 0: Initialize Global Variables
 *********************************************************************************************/
public class BG_SD_Huber_20190128 extends RoboticsAPICyclicBackgroundTask {

	@Inject
	private AnybusIOGroup	IO_EtherCAT;
	@Inject
	private LBR				lBR_iiwa_14_R820_1;
	// private MediaFlangeIOGroup flangeIOs;

	private int				X_Cor_Current_TX;					// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	private int				Y_Cor_Current_TX;					// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	private int				Z_Cor_Current_TX;					// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	private int				W_Cor_Current_TX;					// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	private int				P_Cor_Current_TX;					// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	private int				R_Cor_Current_TX;					// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	private int				Radius_Current_TX;					// Current_radius_position_respect_to_world_frame_to_send_over_Ethernet

	private static double	Radius					= 500;		// Radius of the arc
	private static double	Z_Cor_Request			= 400;		// Z Cord Request
	private static double	Head_Angle;						// Head angle sent to main prog
	private double			Head_Angle_Input;					// Head Angle read from Anybus

	private static boolean	Run_Mode_Enabled_PLC	= false;	// Run Mode status from PLC
	private static boolean	Play_Mode_Enabled_PLC	= false;	// User selected Start on HMI

	private static double	Max_Angle				= 20;		// Max head angle
	private static double	Min_Angle				= -20;		// Min head angle

	/*********************************************************************************************
	 * Step 1: Setup BG Variable Handling To Return Variables
	 *********************************************************************************************/
	public static double get_Radius() {
		return Radius;
	}

	public static double get_Head_Angle() {
		return Head_Angle;
	}

	public static double get_Z_Cor_Request() {
		return Z_Cor_Request;
	}

	public static boolean get_Run_Mode_Enabled_PLC() {
		return Run_Mode_Enabled_PLC;
	}

	public static boolean get_Play_Mode_Enabled_PLC() {
		return Play_Mode_Enabled_PLC;
	}

	public static double get_Max_Angle() {
		return Max_Angle;
	}

	public static double get_Min_Angle() {
		return Min_Angle;
	}

	/*********************************************************************************************
	 * Step 2: Setup BG Variable Handling To Return Variables
	 *********************************************************************************************/
	@Override
	public void initialize() {
		// initialize your task here
		initializeCyclic(0, 25, TimeUnit.MILLISECONDS, CycleBehavior.BestEffort);
	}

	/*********************************************************************************************
	 * Step 3: Main Routine
	 *********************************************************************************************/
	@Override
	public void runCyclic() {

		/*********************************************************************************************
		 * Step 4: Read Current Robot Position
		 *********************************************************************************************/
		Frame Robot_Current_Location = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(lBR_iiwa_14_R820_1.getFlange()); // Get_the_current_cart_position
		/*********************************************************************************************
		 * Step 5: Outputs to PLC
		 *********************************************************************************************/
		// Set negative bit if needed. Could not communicate negative directly.
		if (Robot_Current_Location.getX() < 0) {
			X_Cor_Current_TX = (int) Robot_Current_Location.getX() * -1;
		}
		else {
			X_Cor_Current_TX = (int) Robot_Current_Location.getX();
		}

		if (Robot_Current_Location.getY() < 0) {
			Y_Cor_Current_TX = (int) Robot_Current_Location.getY() * -1;
		}
		else {
			Y_Cor_Current_TX = (int) Robot_Current_Location.getY();
		}

		if (Robot_Current_Location.getZ() < 0) {
			Z_Cor_Current_TX = (int) Robot_Current_Location.getZ() * -1;
		}

		else {
			Z_Cor_Current_TX = (int) Robot_Current_Location.getZ();
		}

		if (Math.toDegrees(Robot_Current_Location.getAlphaRad()) * -1 < 0) {

			W_Cor_Current_TX = (int) Math.toDegrees(Robot_Current_Location.getAlphaRad());
		}
		else {
			W_Cor_Current_TX = (int) Math.toDegrees(Robot_Current_Location.getAlphaRad()) * -1;
		}

		if (Math.toDegrees(Robot_Current_Location.getBetaRad()) * -1 < 0) {
			P_Cor_Current_TX = (int) Math.toDegrees(Robot_Current_Location.getBetaRad());
		}
		else {
			P_Cor_Current_TX = (int) Math.toDegrees(Robot_Current_Location.getBetaRad()) * -1;
		}

		if (Math.toDegrees(Robot_Current_Location.getGammaRad()) * -1 < 0) {
			R_Cor_Current_TX = (int) Math.toDegrees(Robot_Current_Location.getGammaRad());
		}
		else {
			R_Cor_Current_TX = (int) Math.toDegrees(Robot_Current_Location.getGammaRad()) * -1;
		}

		if (Radius < 0) {
			Radius_Current_TX = (int) (Radius);
		}
		else {
			Radius_Current_TX = (int) (Radius);
		}

		// Set the outputs
		IO_EtherCAT.setOutput_0(X_Cor_Current_TX);
		IO_EtherCAT.setOutput_1(Y_Cor_Current_TX);
		IO_EtherCAT.setOutput_2(Z_Cor_Current_TX);
		IO_EtherCAT.setOutput_3(W_Cor_Current_TX);
		IO_EtherCAT.setOutput_4(P_Cor_Current_TX);
		IO_EtherCAT.setOutput_5(R_Cor_Current_TX);
		IO_EtherCAT.setOutput_6(Radius_Current_TX);

		/*********************************************************************************************
		 * Step 6: Read Inputs from PLC
		 *********************************************************************************************/
		// Read in run bit from PLC
		if (BigInteger.valueOf(IO_EtherCAT.getInput_3()).testBit(0)) {
			Run_Mode_Enabled_PLC = true;
		}
		else {
			Run_Mode_Enabled_PLC = false;
		}
		// Read in run bit from PLC
		if (BigInteger.valueOf(IO_EtherCAT.getInput_3()).testBit(1)) {
			Play_Mode_Enabled_PLC = true;
		}
		else {
			Play_Mode_Enabled_PLC = false;
		}

		// Read_in_the_radius_from_the_HMI
		if (IO_EtherCAT.getInput_4() <= 600 && IO_EtherCAT.getInput_6() >= 400) {
			Radius = IO_EtherCAT.getInput_4(); // Read_in_the_radius_from_the_HMI
		}
		else {
			Radius = 500; // Error
		}
		// Read_in_the_camera_angle_from_the_PLC._Shift_over_16_and_check_if_it_is_negative
		// (Range_-32768_to_32767)
		if (32767 < (IO_EtherCAT.getInput_5())) {// Negative_Value_(Range_-32768_to_0)
			// XOR the input with 0xFFFF, add 1 and multiple by negative 1
			Head_Angle_Input = (((IO_EtherCAT.getInput_5()) ^ 65535) + 1) * -1; // Read_in_the_camera_angle_from_the_PLC
		}
		else { // Positive_Value_(Range_0_to_32767)
			Head_Angle_Input = (IO_EtherCAT.getInput_5()); // Read_in_the_camera_angle_from_the_PLC
		}
		// Read_in_the_height_from_the_HMI
		if (IO_EtherCAT.getInput_6() <= 600 && IO_EtherCAT.getInput_6() >= 400) {
			Z_Cor_Request = IO_EtherCAT.getInput_6(); // Read_in_the_height_from_the_HMI
		}
		else {
			Z_Cor_Request = 500; // Error default setting
		}

		/*********************************************************************************************
		 * Step 7: Calculate Max Robot Reach
		 *********************************************************************************************/
		// Calculate max and min head angle
		Max_Angle = (((Radius * -0.1350) + 108.5) * 0.9); // Calculate max head angle. Linear: (Radius = -0.1350 & Head Angle +108.5)
		// Multiplied by 0.9 for a safety factor
		// Radius (mm) Head Angle(deg)
		// 400mm 55deg
		// 500mm 40deg
		// 600mm 28deg

		Min_Angle = Max_Angle * -1; // Minimum angle is inverse
		// ERROR: Head angle was outside of calculated range
		if ((Head_Angle_Input > Max_Angle) || (Head_Angle_Input < Min_Angle)) {
			// Limit the reach
			if (Head_Angle_Input > Max_Angle) {
				Head_Angle = Max_Angle;
			}
			else {
				Head_Angle = Min_Angle;
			}
		}
		else {
			Head_Angle = Head_Angle_Input; //
		}
	}
}