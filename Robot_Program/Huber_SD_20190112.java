/*
 * Author: Nathan Huber
 * Purpose: Senior Design Project
 * Date Started: 20181027
 * Date of Rev: 20181027
 * Revision: 1.0
 * 
 * ---Revision History---
 * Date			Author			Rev			Purpose
 * 20181027		Nathan Huber	1.0			Initial Program
 * 
 * 
 * 
 * 
 */

package Capstone_Huber;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.Random;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/*********************************************************************************************
 * Step 0: Initialize Global Variables
 *********************************************************************************************/
public class Huber_SD_20190112 extends RoboticsAPIApplication {

	private Tool				End_Effector;

	@Inject
	public LBR					lBR_iiwa_14_R820_1;
	public double				X_Cor_Request_Fin		= 600;			// Cartesian_position_respect_to_world_frame
	public double				Y_Cor_Request_Fin		= 0;			// Cartesian_position_respect_to_world_frame
	public double				Z_Cor_Request			= 400;			// Cartesian_position_respect_to_world_frame
	public double				X_Cor_Request_Mid		= 600;			// Cartesian_position_respect_to_world_frame_requested_second
	public double				Y_Cor_Request_Mid		= 0;			// Cartesian_position_respect_to_world_frame_requested_second

	double[]					Home_Zero				= new double[] { 0, Math.toRadians(25), 0, Math.toRadians(-100), 0, Math.toRadians(-60),
			Math.toRadians(0)							};
	public int					X_Cor_Current_TX;						// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	public int					Y_Cor_Current_TX;						// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	public int					Z_Cor_Current_TX;						// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	public int					W_Cor_Current_TX;						// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	public int					P_Cor_Current_TX;						// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	public int					R_Cor_Current_TX;						// Current_Cartesian_position_respect_to_world_frame_to_send_over_Ethernet
	public int					Radius_Current_TX;						// Current_radius_position_respect_to_world_frame_to_send_over_Ethernet

	public double				Roll_Coor_Request_Mid;
	public double				Roll_Coor_Current		= 0;			// Request cart position for head angle. Incremented by Rotational_Smoothness
	public double				Radius					= 500;			// Radius of the arc
	public double				X_Offset				= 200;			// Protect the robot from going in to far
	public double				Head_Angle;
	public double				Dis_From_Limits			= 0;			// Difference between current head angle position and max.

	public int					X_Stiffness				= 750;			// Stiffness of the robot
	public int					Y_Stiffness				= 750;			// Stiffness of the robot
	public int					Z_Stiffness				= 2000;		// Stiffness of the robot

	public boolean				Run_Mode_Enabled		= false;		// PLC Says okay to Run
	public boolean				Robot_Angle_Change		= false;		// Change in angle of the head

	public int					Random_Angle			= 25;			// Random Angle value
	public double				Angle_Stop_Threshold	= 2;			// Threshold
	public boolean				Axis_Torque_Ref;						// All Axis torque reference state
	public boolean				Axis_Pos_Ref;							// All Axis position reference state

	public String				Information_Display		= "";			// Message for console to display
	public int					ERROR_MSG;								// Error message
	Random						random					= new Random(); // Setup random value

	@Inject
	private MediaFlangeIOGroup	flangeIOs;								// Flange inputs and outputs

	/*********************************************************************************************
	 * Step 1: Setup BG Programs & Attach Tool
	 *********************************************************************************************/
	@Override
	public void initialize() {
		getLogger().info("Running!"); // Say we are running
		End_Effector = getApplicationData().createFromTemplate("Control_Panel"); // Attach object to end effector
		End_Effector.attachTo(lBR_iiwa_14_R820_1.getFlange()); // Attach object to end effector

		// Make sure joints referenced
		Axis_Torque_Ref = lBR_iiwa_14_R820_1.getSafetyState().areAllAxesGMSReferenced(); // Check the status of the torque ref
		Axis_Pos_Ref = lBR_iiwa_14_R820_1.getSafetyState().areAllAxesPositionReferenced(); // Check the status of the position ref
		// Make sure axis are referenced
		if (!Axis_Torque_Ref || !Axis_Pos_Ref) {
			lBR_iiwa_14_R820_1.move(ptp(0, 0, 0, 0, 0, 0, 0)); // Move to zero position
			getApplicationUI().displayModalDialog(ApplicationDialogType.ERROR, "ERROR: RUN PositionAndGMSReferencing.java", "OK"); // Tell the user we need to run the program to continue
			getApplicationControl().halt(); // Stop the program
		}
	}

	/*********************************************************************************************
	 * Step 2: Main Routine
	 *********************************************************************************************/
	@Override
	public void run() {

		/*********************************************************************************************
		 * Step 3: Zero some joints
		 *********************************************************************************************/
		Frame Robot_Pos_Frame_0 = getApplicationData().getFrame("/SD_Frame").copy(); // Store_Home_Position_For_Later
		Frame Robot_Pos_Frame_1 = getApplicationData().getFrame("/SD_Frame").copy(); // Store_Home_Position_For_Later
		// Step_2: Move to start position
		PTP Joint_Zero = ptp(Home_Zero);// Zero axis
		getLogger().info("Zero some joints"); // Tell the user we are zeroing a few joints
		lBR_iiwa_14_R820_1.move(Joint_Zero); // Make some joints zero
		/*********************************************************************************************
		 * Step 4: Move Home
		 *********************************************************************************************/
		getLogger().info("Move to start position");// Tell user we are going to start position
		lBR_iiwa_14_R820_1.move(ptp(getApplicationData().getFrame("/SD_Frame"))); // Move to start position

		/*********************************************************************************************
		 * Step 5: Prompt User for Mode Select
		 *********************************************************************************************/
		int UI_Request_Demo = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Run demo program?", "Sweep", "Random", "Play",
				"No");
		getLogger().info("BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC(): " + BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC()); // What mode does the user want to run in
		/*********************************************************************************************
		 * Step 6: Execute Selected Program
		 *********************************************************************************************/
		switch (UI_Request_Demo) {
		/*********************************************************************************************
		 * Step 7A: Sweep Mode
		 *********************************************************************************************/
			case 0: // *** Sweep Mode ***
				while (BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC()) {
					// Loop from XXX to Max Angle
					getLogger().info("*** SWEEP MAX ***");
					Play_Routine(Robot_Pos_Frame_0, Robot_Pos_Frame_1, BG_SD_Huber_20190128.get_Max_Angle()); // Move to the max angle
					// Loop from Max Angle to Min Angle
					getLogger().info("*** SWEEP MIN ***");
					Play_Routine(Robot_Pos_Frame_0, Robot_Pos_Frame_1, BG_SD_Huber_20190128.get_Min_Angle()); // Move to the minimum angle
					getLogger().info("BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC(): " + BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC());
				}
				/*********************************************************************************************
				 * Step 7B: Random Mode
				 *********************************************************************************************/
			case 1: // *** Random Mode ***
				while (BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC()) {
					// ***Get_current_location***
					Frame Robot_Current_Location = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(lBR_iiwa_14_R820_1.getFlange()); // Get_the_current_cart_position
					Roll_Coor_Current = Math.toDegrees(Robot_Current_Location.getAlphaRad() * -1); // Store_the_temporary_location_and_(inc_or_dec)
					// ***Generate_a_random_value_between_Min_&_Max_angle***
					Random_Angle = (int) (BG_SD_Huber_20190128.get_Max_Angle() + (BG_SD_Huber_20190128.get_Max_Angle() - BG_SD_Huber_20190128
							.get_Min_Angle()) * random.nextDouble()); // Generate random angle
					getLogger().info("Random_Angle: " + Random_Angle); // Display Random angle
					// ***_Run_the_"Play_Routine" ***
					Play_Routine(Robot_Pos_Frame_0, Robot_Pos_Frame_1, Random_Angle); // Move the robot
					getLogger().info("*** NEXT RANDOM VALUE ***"); // Display Random angle
				}
				/*********************************************************************************************
				 * Step 7C: Play Mode
				 *********************************************************************************************/
			case 2: // *** Play mode ***
				while (BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC()) {
					// Run if user selected start on HMI
					while (BG_SD_Huber_20190128.get_Play_Mode_Enabled_PLC()) {
						Play_Routine(Robot_Pos_Frame_0, Robot_Pos_Frame_1, BG_SD_Huber_20190128.get_Head_Angle()); // Move the robot
						if (!BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC()) {
							break; // Stop program
						}
					}
				}
			case 3:
				while (true)
					return;
		}
	}

	/*********************************************************************************************
	 * Play Routine
	 *********************************************************************************************/
	public void Play_Routine(Frame Robot_Pos_Frame_0, Frame Robot_Pos_Frame_1, double Roll_Coor_Request_Fin) {

		// Step_8: Verify OK To Run
		// Step_9: Get the current location & store the current head angle (Roll_Coor_Current)
		// Step_10: Read in any radius & Z height change requests
		// Step_11: Proceed if angle differs from current
		// Step_12: Calculate the mid point and final point on the path
		// Step_13: Setup the impedance control of the motion
		// Step_14: Setup the frame with the mid and final points
		// Step_15: Display our wealth of knowledge
		// Step_16: Move the robot with a circ motion command

		/*********************************************************************************************
		 * Step 8: Verify OK To Run
		 *********************************************************************************************/
		if (BG_SD_Huber_20190128.get_Run_Mode_Enabled_PLC()) {
			/*********************************************************************************************
			 * Step 9: Get the current location & store the current head angle (Roll_Coor_Current)
			 *********************************************************************************************/
			Frame Robot_Current_Location = lBR_iiwa_14_R820_1.getCurrentCartesianPosition(lBR_iiwa_14_R820_1.getFlange()); // Get_the_current_cart_position
			Roll_Coor_Current = Math.toDegrees(Robot_Current_Location.getAlphaRad()) * -1; // Store_the_temporary_location_and_(inc_or_dec)
			/*********************************************************************************************
			 * Step 10: Read in any radius & Z height change requests
			 *********************************************************************************************/
			Radius = BG_SD_Huber_20190128.get_Radius(); // Read in the current radius
			Z_Cor_Request = BG_SD_Huber_20190128.get_Z_Cor_Request(); // Read in the Z Request
			/*********************************************************************************************
			 * Step 11: Proceed if angle differs from current
			 *********************************************************************************************/
			if ((Roll_Coor_Request_Fin <= Roll_Coor_Current + (Angle_Stop_Threshold))
					&& (Roll_Coor_Request_Fin >= Roll_Coor_Current - (Angle_Stop_Threshold))) {
				Robot_Angle_Change = false; // No need to move
				flangeIOs.setLEDBlue(false); // Indicate
			}
			else {
				Robot_Angle_Change = true; // Move
				flangeIOs.setLEDBlue(true);// Indicate
			}

			if (Robot_Angle_Change) {
				getLogger().info("*************************************"); // Add some separation on the terminal
				/*********************************************************************************************
				 * Step 12: Calculate the mid point and final point on the path
				 *********************************************************************************************/
				Roll_Coor_Request_Mid = (Roll_Coor_Request_Fin - (Roll_Coor_Request_Fin - (Math.toDegrees(Robot_Current_Location.getAlphaRad()) * -1)) * 0.5);

				X_Cor_Request_Fin = ((Radius - (Radius * Math.cos((Roll_Coor_Request_Fin * Math.PI) / 180))) + Radius) + X_Offset; // Calculate_the_X_position_&_mirror_it_to_arc_out
				Y_Cor_Request_Fin = (Radius * Math.sin((Roll_Coor_Request_Fin * Math.PI) / 180)); // Calculate_the_Y_position

				X_Cor_Request_Mid = ((Radius - (Radius * Math.cos((Roll_Coor_Request_Mid * Math.PI) / 180))) + Radius) + X_Offset; // Calculate_the_X_position_&_mirror_it_to_arc_out
				Y_Cor_Request_Mid = (Radius * Math.sin((Roll_Coor_Request_Mid * Math.PI) / 180)); // Calculate_the_Y_position

				/*********************************************************************************************
				 * Step 13: Setup the impedance control of the motion
				 *********************************************************************************************/
				CartesianImpedanceControlMode cartImpCtrlMode = new CartesianImpedanceControlMode();
				cartImpCtrlMode.parametrize(CartDOF.X).setStiffness(X_Stiffness);
				cartImpCtrlMode.parametrize(CartDOF.Y).setStiffness(Y_Stiffness);
				cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(Z_Stiffness);

				/*********************************************************************************************
				 * Step 14: Setup the frame with the mid and final points
				 *********************************************************************************************/
				Frame Robot_Pos_Req = (Robot_Pos_Frame_0.setX(X_Cor_Request_Fin).setY(Y_Cor_Request_Fin).setZ(Z_Cor_Request).setAlphaRad(Math
						.toRadians(-Roll_Coor_Request_Fin))); // Position of first point
				Frame Robot_Pos_Req_Mid = (Robot_Pos_Frame_1.setX(X_Cor_Request_Mid).setY(Y_Cor_Request_Mid).setZ(Z_Cor_Request).setAlphaRad(Math
						.toRadians(-Roll_Coor_Request_Mid))); // Position of second point
				/*********************************************************************************************
				 * Step 15: Display our wealth of knowledge
				 *********************************************************************************************/
				Information_Display = ("\n" + "Roll_Coor_Request_Fin: " + Roll_Coor_Request_Fin + "\n" + "Robot_Pos_Req: " + Robot_Pos_Req + "\n"
						+ "Robot_Pos_Req_Mid: " + Robot_Pos_Req_Mid + "\n" + "Robot_Current_Location: " + Robot_Current_Location);
				Information_Display = Information_Display
						+ ("\n" + "Head Angle: " + Roll_Coor_Current + "\n" + "X_Cor_Request_Fin: " + X_Cor_Request_Fin + "\n"
								+ "Y_Cor_Request_Fin: " + Y_Cor_Request_Fin + "\n" + "Roll_Coor_Current: " + Roll_Coor_Current);
				Information_Display = Information_Display
						+ ("\n" + "Roll_Coor_Request_Fin: " + Roll_Coor_Request_Fin + "\n" + "Robot_Angle_Change: " + Robot_Angle_Change + "\n");
				getLogger().info(Information_Display);
				/*********************************************************************************************
				 * Step 16: Move the robot with a circ motion command
				 *********************************************************************************************/
				try {
					lBR_iiwa_14_R820_1.move(circ(Robot_Pos_Req_Mid, Robot_Pos_Req)); // Move robot
				}
				catch (Exception e) {
					getLogger().info("\n" + "!!!!!!!!!!!!!!!!!" + "\n" + "Could not plan motion path." + "\n" + "!!!!!!!!!!!!!!!!!" + "\n");
					ThreadUtil.milliSleep(750); // Make sure we don't get stuck in a while loop
					return;
				}

			}
		}
	}
}
